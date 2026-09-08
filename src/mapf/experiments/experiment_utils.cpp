#include "experiment_utils.hpp"

#include "mapf/utils.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <unordered_set>
#include <utility>

#ifndef MAPF_REPOSITORY_ROOT
#define MAPF_REPOSITORY_ROOT "."
#endif

namespace mapf::experiments {

    namespace {
        struct ConflictKey {
            int timestep;
            ConflictType type;
            int cell1X;
            int cell1Y;
            int cell2X;
            int cell2Y;

            bool operator<(const ConflictKey& other) const {
                return std::tie(
                    timestep,
                    type,
                    cell1X,
                    cell1Y,
                    cell2X,
                    cell2Y
                ) < std::tie(
                    other.timestep,
                    other.type,
                    other.cell1X,
                    other.cell1Y,
                    other.cell2X,
                    other.cell2Y
                );
            }
        };

        struct ConflictAccumulator {
            Cell* cell1;
            Cell* cell2;
            std::set<int> agentIds;
        };

        bool cellComesBefore(const Cell* first, const Cell* second) {
            return std::tie(first->position.x, first->position.y) <
                std::tie(second->position.x, second->position.y);
        }

        Cell* positionAt(const std::vector<Cell*>& path, int timestep) {
            if (timestep < static_cast<int>(path.size())) {
                return path[timestep];
            }

            return path.back();
        }

        std::string sanitizeFilenameComponent(const std::string& input) {
            std::string output;
            output.reserve(input.size());

            for (char character : input) {
                const unsigned char unsignedCharacter =
                    static_cast<unsigned char>(character);
                if (
                    std::isalnum(unsignedCharacter) ||
                    character == '.' ||
                    character == '_' ||
                    character == '-'
                ) {
                    output += character;
                } else {
                    output += '_';
                }
            }

            return output.empty() ? "unknown" : output;
        }

        std::string currentBranchName() {
            std::ifstream headFile(repositoryRoot() / ".git" / "HEAD");
            if (!headFile.is_open()) {
                return "unknown";
            }

            std::string head;
            std::getline(headFile, head);

            constexpr const char* refPrefix = "ref: refs/heads/";
            const std::string prefix(refPrefix);
            if (head.rfind(prefix, 0) == 0) {
                return sanitizeFilenameComponent(head.substr(prefix.size()));
            }

            return "detached";
        }

        long long nextTimestampMilliseconds() {
            static std::mutex timestampMutex;
            static long long lastTimestamp = -1;

            const long long clockTimestamp =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count();

            std::lock_guard lock(timestampMutex);
            lastTimestamp = std::max(clockTimestamp, lastTimestamp + 1);
            return lastTimestamp;
        }

        std::string formatDouble(double value) {
            std::ostringstream output;
            output << std::fixed << std::setprecision(6) << value;
            return output.str();
        }

        std::string formatPosition(const Position& position) {
            return std::to_string(position.x) + "-" + std::to_string(position.y);
        }

        std::string formatCell(const Cell* cell) {
            if (cell == nullptr) {
                return "";
            }

            return formatPosition(cell->position);
        }

        std::string formatPath(const std::list<Cell*>& path) {
            std::ostringstream output;
            bool first = true;

            for (Cell* cell : path) {
                if (!first) {
                    output << '|';
                }

                output << formatCell(cell);
                first = false;
            }

            return output.str();
        }

        int artifactPathCost(const std::list<Cell*>& path) {
            return path.empty() ? -1 : static_cast<int>(path.size()) - 1;
        }

        double calculateInjustice(
            const std::vector<std::list<Cell*>>& initialPaths,
            const std::vector<std::list<Cell*>>& solutionPaths
        ) {
            std::vector<double> differences;
            differences.reserve(initialPaths.size());
            double sum = 0.0;

            for (std::size_t i = 0; i < initialPaths.size(); i++) {
                if (initialPaths[i].empty() || solutionPaths[i].empty()) {
                    continue;
                }

                const double difference = static_cast<double>(
                    artifactPathCost(solutionPaths[i]) -
                    artifactPathCost(initialPaths[i])
                );
                differences.push_back(difference);
                sum += difference;
            }

            if (differences.empty()) {
                return 0.0;
            }

            const double mean = sum / static_cast<double>(differences.size());
            double squaredDistanceSum = 0.0;
            for (double difference : differences) {
                const double distance = difference - mean;
                squaredDistanceSum += distance * distance;
            }

            return std::sqrt(
                squaredDistanceSum / static_cast<double>(differences.size())
            );
        }

        bool supportedSolver(const std::string& solver) {
            return
                solver == "PriorityPlanningSolver" ||
                solver == "LocalPathRepairParallelSolver" ||
                solver == "LocalPathRepairIterativeSolver";
        }

        bool validSolverMetadata(const ExperimentRunResult& run) {
            if (run.solver == "PriorityPlanningSolver") {
                return
                    !run.continueIfFailed &&
                    !run.multithreading &&
                    run.numThreads == 1 &&
                    !run.localRepair;
            }

            if (run.solver == "LocalPathRepairParallelSolver") {
                return run.multithreading && run.numThreads > 0 && run.localRepair;
            }

            if (run.solver == "LocalPathRepairIterativeSolver") {
                return !run.multithreading && run.numThreads == 1 && run.localRepair;
            }

            return false;
        }

        std::string joinAgentIds(const std::vector<int>& agentIds) {
            std::ostringstream output;
            for (std::size_t i = 0; i < agentIds.size(); i++) {
                if (i > 0) {
                    output << '|';
                }
                output << agentIds[i];
            }
            return output.str();
        }

        void removeDirectory(const std::filesystem::path& directory) {
            std::error_code ignored;
            std::filesystem::remove_all(directory, ignored);
        }
    }

    std::filesystem::path repositoryRoot() {
        return std::filesystem::path(MAPF_REPOSITORY_ROOT);
    }

    std::filesystem::path makeResultPath() {
        const auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();
        return repositoryRoot() / "results" /
            (currentBranchName() + "_" + std::to_string(timestamp) + "_results.csv");
    }

    ExperimentOutputPaths makeExperimentOutputPaths() {
        const std::string prefix = currentBranchName() + "_" +
            std::to_string(nextTimestampMilliseconds());
        const std::filesystem::path directory = repositoryRoot() / "results" / prefix;

        return {
            .prefix = prefix,
            .directory = directory,
            .stats = directory / (prefix + "_stats.csv"),
            .solution = directory / (prefix + "_solution.csv"),
            .conflicts = directory / (prefix + "_conflicts.csv")
        };
    }

    double elapsedSeconds(std::chrono::steady_clock::time_point startedAt) {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - startedAt
        ).count();
    }

    std::vector<ConflictRecord> normalizeConflicts(
        const Instance& instance,
        const std::vector<std::list<Cell*>>& paths
    ) {
        const std::vector<Agent>& agents = instance.getAgents();
        if (paths.size() != agents.size()) {
            throw std::invalid_argument("Paths must be aligned with instance agents.");
        }

        std::vector<std::vector<Cell*>> indexedPaths;
        indexedPaths.reserve(paths.size());
        int makespan = 0;

        for (const std::list<Cell*>& path : paths) {
            indexedPaths.emplace_back(path.begin(), path.end());
            if (!path.empty()) {
                makespan = std::max(makespan, static_cast<int>(path.size()) - 1);
            }
        }

        std::map<ConflictKey, ConflictAccumulator> conflicts;

        for (int timestep = 0; timestep <= makespan; timestep++) {
            for (std::size_t firstIndex = 0; firstIndex < indexedPaths.size(); firstIndex++) {
                if (indexedPaths[firstIndex].empty()) {
                    continue;
                }

                for (
                    std::size_t secondIndex = firstIndex + 1;
                    secondIndex < indexedPaths.size();
                    secondIndex++
                ) {
                    if (indexedPaths[secondIndex].empty()) {
                        continue;
                    }

                    Cell* firstCell = positionAt(indexedPaths[firstIndex], timestep);
                    Cell* secondCell = positionAt(indexedPaths[secondIndex], timestep);

                    if (firstCell == secondCell) {
                        const ConflictKey key {
                            .timestep = timestep,
                            .type = ConflictType::Vertex,
                            .cell1X = firstCell->position.x,
                            .cell1Y = firstCell->position.y,
                            .cell2X = firstCell->position.x,
                            .cell2Y = firstCell->position.y
                        };
                        auto [conflict, inserted] = conflicts.try_emplace(
                            key,
                            ConflictAccumulator {
                                .cell1 = firstCell,
                                .cell2 = firstCell,
                                .agentIds = {}
                            }
                        );
                        (void) inserted;
                        conflict->second.agentIds.insert(agents[firstIndex].id);
                        conflict->second.agentIds.insert(agents[secondIndex].id);
                    }

                    if (timestep == 0) {
                        continue;
                    }

                    Cell* firstPrevious = positionAt(indexedPaths[firstIndex], timestep - 1);
                    Cell* secondPrevious = positionAt(indexedPaths[secondIndex], timestep - 1);
                    const bool oppositeDirections =
                        firstPrevious != firstCell &&
                        secondPrevious != secondCell &&
                        firstPrevious == secondCell &&
                        firstCell == secondPrevious;
                    if (!oppositeDirections) {
                        continue;
                    }

                    Cell* edgeFirst = firstPrevious;
                    Cell* edgeSecond = firstCell;
                    if (cellComesBefore(edgeSecond, edgeFirst)) {
                        std::swap(edgeFirst, edgeSecond);
                    }

                    const ConflictKey key {
                        .timestep = timestep,
                        .type = ConflictType::Edge,
                        .cell1X = edgeFirst->position.x,
                        .cell1Y = edgeFirst->position.y,
                        .cell2X = edgeSecond->position.x,
                        .cell2Y = edgeSecond->position.y
                    };
                    auto [conflict, inserted] = conflicts.try_emplace(
                        key,
                        ConflictAccumulator {
                            .cell1 = edgeFirst,
                            .cell2 = edgeSecond,
                            .agentIds = {}
                        }
                    );
                    (void) inserted;
                    conflict->second.agentIds.insert(agents[firstIndex].id);
                    conflict->second.agentIds.insert(agents[secondIndex].id);
                }
            }
        }

        std::vector<ConflictRecord> records;
        records.reserve(conflicts.size());
        for (const auto& [key, conflict] : conflicts) {
            records.push_back({
                .cell1 = conflict.cell1,
                .cell2 = conflict.cell2,
                .timestep = key.timestep,
                .type = key.type,
                .agentIds = std::vector<int>(
                    conflict.agentIds.begin(),
                    conflict.agentIds.end()
                )
            });
        }

        return records;
    }

    bool writeExperimentArtifacts(
        const Instance& instance,
        const ExperimentRunResult& run,
        double experimentTimeSeconds
    ) {
        const std::vector<Agent>& agents = instance.getAgents();
        if (
            run.initialPaths.size() != agents.size() ||
            run.solutionPaths.size() != agents.size() ||
            run.numAgents < 0 ||
            static_cast<std::size_t>(run.numAgents) != agents.size() ||
            !supportedSolver(run.solver) ||
            !validSolverMetadata(run)
        ) {
            return false;
        }

        std::vector<ConflictRecord> finalConflicts;
        try {
            finalConflicts = normalizeConflicts(instance, run.solutionPaths);
        } catch (const std::exception&) {
            return false;
        }

        std::unordered_set<int> conflictedAgentIds;
        for (const ConflictRecord& conflict : finalConflicts) {
            conflictedAgentIds.insert(
                conflict.agentIds.begin(),
                conflict.agentIds.end()
            );
        }

        int pathsResolved = 0;
        int sumOfCosts = 0;
        int makespan = 0;
        CsvRow solutionHeaders = {
            "agent_id",
            "agent_scenario_bucket",
            "start",
            "goal",
            "optimum_path",
            "solution_path",
            "optimum_path_cost",
            "solution_path_cost",
            "success_optimum_path",
            "success_solution_path"
        };
        std::vector<CsvRow> solutionRows;
        solutionRows.reserve(agents.size());

        for (std::size_t i = 0; i < agents.size(); i++) {
            const Agent& agent = agents[i];
            const std::list<Cell*>& initialPath = run.initialPaths[i];
            const std::list<Cell*>& solutionPath = run.solutionPaths[i];
            const int optimumPathCost = artifactPathCost(initialPath);
            const int solutionPathCost = artifactPathCost(solutionPath);
            const bool successOptimumPath = !initialPath.empty();
            const bool successSolutionPath =
                !solutionPath.empty() &&
                !conflictedAgentIds.contains(agent.id);

            if (!solutionPath.empty()) {
                sumOfCosts += solutionPathCost;
                makespan = std::max(makespan, solutionPathCost);
            }
            if (successSolutionPath) {
                pathsResolved++;
            }

            solutionRows.push_back({
                std::to_string(agent.id),
                std::to_string(agent.scenarioId),
                formatPosition(agent.startPosition),
                formatPosition(agent.goalPosition),
                formatPath(initialPath),
                formatPath(solutionPath),
                std::to_string(optimumPathCost),
                std::to_string(solutionPathCost),
                successOptimumPath ? "true" : "false",
                successSolutionPath ? "true" : "false"
            });
        }

        const CsvRow statsHeaders = {
            "map",
            "instance_name",
            "num_agents",
            "success",
            "paths_resolved",
            "sumOfCosts",
            "makespan",
            "injustice",
            "durationSeconds",
            "time",
            "multithreading",
            "num_threads",
            "solver",
            "continue_if_failed"
        };
        const CsvRow statsRow = {
            instance.getMapName(),
            instance.getInstanceName(),
            std::to_string(run.numAgents),
            run.metrics.success ? "true" : "false",
            std::to_string(pathsResolved),
            std::to_string(sumOfCosts),
            std::to_string(makespan),
            formatDouble(calculateInjustice(run.initialPaths, run.solutionPaths)),
            formatDouble(run.metrics.durationSeconds),
            formatDouble(experimentTimeSeconds),
            run.multithreading ? "true" : "false",
            std::to_string(run.numThreads),
            run.solver,
            run.continueIfFailed ? "true" : "false"
        };

        const CsvRow conflictHeaders = {
            "cell_1",
            "cell_2",
            "timestep",
            "conflict_type",
            "agents"
        };
        std::vector<CsvRow> conflictRows;
        conflictRows.reserve(finalConflicts.size());
        for (const ConflictRecord& conflict : finalConflicts) {
            conflictRows.push_back({
                formatCell(conflict.cell1),
                formatCell(conflict.cell2),
                std::to_string(conflict.timestep),
                conflict.type == ConflictType::Vertex ? "vertex" : "edge",
                joinAgentIds(conflict.agentIds)
            });
        }

        const ExperimentOutputPaths outputPaths = makeExperimentOutputPaths();
        const std::filesystem::path resultsDirectory = outputPaths.directory.parent_path();
        const std::filesystem::path temporaryDirectory =
            resultsDirectory / ("." + outputPaths.prefix + ".tmp");
        const std::filesystem::path temporaryStats =
            temporaryDirectory / outputPaths.stats.filename();
        const std::filesystem::path temporarySolution =
            temporaryDirectory / outputPaths.solution.filename();
        const std::filesystem::path temporaryConflicts =
            temporaryDirectory / outputPaths.conflicts.filename();

        std::error_code errorCode;
        std::filesystem::create_directories(resultsDirectory, errorCode);
        if (errorCode) {
            return false;
        }

        if (!std::filesystem::create_directory(temporaryDirectory, errorCode) || errorCode) {
            return false;
        }

        const bool needsConflicts = run.localRepair && !run.metrics.success;
        const bool wroteAll =
            writeRowsToCsvFile(temporaryStats, statsHeaders, {statsRow}) &&
            writeRowsToCsvFile(temporarySolution, solutionHeaders, solutionRows) &&
            (!needsConflicts || writeRowsToCsvFile(
                temporaryConflicts,
                conflictHeaders,
                conflictRows
            ));
        if (!wroteAll) {
            removeDirectory(temporaryDirectory);
            return false;
        }

        std::filesystem::rename(temporaryDirectory, outputPaths.directory, errorCode);
        if (errorCode) {
            removeDirectory(temporaryDirectory);
            return false;
        }

        return true;
    }

    bool writeExperimentResult(
        const Instance& instance,
        const Result& result,
        double experimentTimeSeconds
    ) {
        const CsvRow headers = {
            "map",
            "instance_name",
            "num_agents",
            "success",
            "sumOfCosts",
            "makespan",
            "injustice",
            "durationSeconds",
            "time"
        };
        const CsvRow row = {
            instance.getMapName(),
            instance.getInstanceName(),
            std::to_string(instance.getNumAgents()),
            result.success ? "true" : "false",
            std::to_string(result.sumOfCosts),
            std::to_string(result.makespan),
            formatDouble(result.injustice),
            formatDouble(result.durationSeconds),
            formatDouble(experimentTimeSeconds)
        };

        return writeRowsToCsvFile(makeResultPath(), headers, {row});
    }

}
