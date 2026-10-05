#include "experiment_utils.hpp"

#include "mapf/utils.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <mutex>
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
                solver == "LocalPathRepairIterativeSolver" ||
                solver == "FullPathRepairIterativeSolver";
        }

        std::string strategyName(LocalRepairStrategy strategy) {
            switch (strategy) {
            case LocalRepairStrategy::RESOLVE_BY_AGENT:
                return "RESOLVE_BY_AGENT";
            case LocalRepairStrategy::RESOLVE_BY_TIME:
                return "RESOLVE_BY_TIME";
            default:
                return "";
            }
        }

        bool validSolverMetadata(const ExperimentRunResult& run) {
            if (run.solver == "PriorityPlanningSolver" || run.solver == "FullPathRepairIterativeSolver") {
                return
                    !run.continueIfFailed &&
                    !run.multithreading &&
                    run.numThreads == 1 &&
                    !run.localRepair && !run.localRepairStrategy;
            }

            if (strategyName(run.localRepairStrategy.value_or(
                LocalRepairStrategy::RESOLVE_BY_AGENT)).empty()) {
                return false;
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

        const SolutionConflicts snapshot = getCollision(paths);
        std::vector<ConflictRecord> records;
        records.reserve(snapshot.vertexEvents.size() + snapshot.edgeEvents.size());
        for (const std::pair<const CellTime, VertexEvent>& entry : snapshot.vertexEvents) {
            std::vector<int> agentIds;
            agentIds.reserve(entry.second.participants.size());
            for (int pathIndex : entry.second.participants) {
                agentIds.push_back(agents[static_cast<std::size_t>(pathIndex)].id);
            }
            records.push_back({entry.first.cell, entry.first.cell, entry.first.time,
                ConflictType::Vertex, std::move(agentIds)});
        }
        for (const std::pair<const EdgeTime, EdgeEvent>& entry : snapshot.edgeEvents) {
            std::vector<int> agentIds;
            agentIds.reserve(entry.second.forward.size() + entry.second.reverse.size());
            for (int pathIndex : entry.second.forward) {
                agentIds.push_back(agents[static_cast<std::size_t>(pathIndex)].id);
            }
            for (int pathIndex : entry.second.reverse) {
                agentIds.push_back(agents[static_cast<std::size_t>(pathIndex)].id);
            }
            records.push_back({entry.first.first, entry.first.second, entry.first.time,
                ConflictType::Edge, std::move(agentIds)});
        }
        for (ConflictRecord& record : records) {
            std::sort(record.agentIds.begin(), record.agentIds.end());
            record.agentIds.erase(std::unique(record.agentIds.begin(), record.agentIds.end()),
                record.agentIds.end());
        }
        std::sort(records.begin(), records.end(),
            [](const ConflictRecord& first, const ConflictRecord& second) {
                return std::tie(first.timestep, first.type, first.cell1->position.x,
                    first.cell1->position.y, first.cell2->position.x, first.cell2->position.y) <
                    std::tie(second.timestep, second.type, second.cell1->position.x,
                    second.cell1->position.y, second.cell2->position.x, second.cell2->position.y);
            });

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
        std::vector<std::string> solutionHeaders = {
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
        std::vector<std::vector<std::string>> solutionRows;
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

        const std::vector<std::string> statsHeaders = {
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
            "continue_if_failed",
            "local_repair_strategy"
        };
        const std::vector<std::string> statsRow = {
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
            run.continueIfFailed ? "true" : "false",
            run.localRepair ? strategyName(run.localRepairStrategy.value_or(
                LocalRepairStrategy::RESOLVE_BY_AGENT)) : "-"
        };

        const std::vector<std::string> conflictHeaders = {
            "cell_1",
            "cell_2",
            "timestep",
            "conflict_type",
            "agents"
        };
        std::vector<std::vector<std::string>> conflictRows;
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

        const bool needsConflicts = !run.metrics.success &&
            (run.localRepair || run.solver == "FullPathRepairIterativeSolver");
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
