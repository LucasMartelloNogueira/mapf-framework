#include "experiment_utils.hpp"

#include "mapf/utils.hpp"
#include "test_support.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <string>
#include <vector>

namespace {
    mapf::Agent agent(int id, int startX, int startY, int goalX, int goalY) {
        mapf::Position start {.x = startX, .y = startY};
        mapf::Position goal {.x = goalX, .y = goalY};
        return {
            .id = id,
            .currentPosition = start,
            .startPosition = start,
            .goalPosition = goal
        };
    }

    std::set<std::filesystem::path> resultDirectories() {
        std::set<std::filesystem::path> directories;
        const std::filesystem::path results =
            mapf::experiments::repositoryRoot() / "results";
        if (!std::filesystem::exists(results)) {
            return directories;
        }

        for (const std::filesystem::directory_entry& entry :
            std::filesystem::directory_iterator(results)) {
            if (entry.is_directory()) {
                directories.insert(entry.path());
            }
        }
        return directories;
    }

    std::filesystem::path onlyNewDirectory(
        const std::set<std::filesystem::path>& before,
        const std::set<std::filesystem::path>& after
    ) {
        std::vector<std::filesystem::path> difference;
        for (const std::filesystem::path& path : after) {
            if (!before.contains(path)) {
                difference.push_back(path);
            }
        }
        requireTest(difference.size() == 1, "Expected exactly one new experiment directory.");
        return difference.front();
    }

    std::string readFile(const std::filesystem::path& path) {
        std::ifstream input(path);
        return std::string(
            std::istreambuf_iterator<char>(input),
            std::istreambuf_iterator<char>()
        );
    }
}

int main() {
    // Scenario: three agents reach one vertex together. Expected: pairwise events normalize into one record with every sorted experiment ID.
    {
        std::vector<std::vector<int>> freeCells(2, std::vector<int>(3, 1));
        std::vector<mapf::Agent> agents {
            agent(8, 0, 0, 1, 0),
            agent(2, 2, 0, 1, 0),
            agent(5, 1, 1, 1, 0)
        };
        mapf::Instance instance(&freeCells, 2, 3, agents);
        mapf::Cell* center = instance.getGrid().getCellPtr(1, 0);
        std::vector<std::list<mapf::Cell*>> paths {
            {instance.getGrid().getCellPtr(0, 0), center},
            {instance.getGrid().getCellPtr(2, 0), center},
            {instance.getGrid().getCellPtr(1, 1), center}
        };

        std::vector<mapf::experiments::ConflictRecord> conflicts =
            mapf::experiments::normalizeConflicts(instance, paths);
        requireTest(conflicts.size() == 1, "Vertex conflicts were not normalized.");
        requireTest(
            conflicts[0].agentIds == std::vector<int>({2, 5, 8}),
            "Normalized agent IDs are incomplete or unsorted."
        );
    }

    // Scenario: two output descriptors are requested immediately. Expected: both use nested run paths and distinct millisecond prefixes.
    {
        const mapf::experiments::ExperimentOutputPaths first =
            mapf::experiments::makeExperimentOutputPaths();
        const mapf::experiments::ExperimentOutputPaths second =
            mapf::experiments::makeExperimentOutputPaths();
        requireTest(first.prefix != second.prefix, "Output prefixes collided.");
        requireTest(
            first.stats.parent_path() == first.directory &&
                first.solution.parent_path() == first.directory &&
                first.conflicts.parent_path() == first.directory,
            "Artifacts are not nested in the run directory."
        );
        requireTest(
            first.stats.filename() == first.prefix + "_stats.csv",
            "Stats filename does not repeat the run prefix."
        );
    }

    // Scenario: a rectangular CSV contains commas and quotes. Expected: the generalized writer escapes fields and rejects malformed row widths.
    {
        const std::filesystem::path path =
            std::filesystem::temp_directory_path() /
            ("mapf_csv_test_" + std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()
            ) + ".csv");
        requireTest(
            writeRowsToCsvFile(path, {"first", "second"}, {{"a,b", "c\"d"}}),
            "Rectangular CSV write failed."
        );
        requireTest(
            readFile(path) == "first,second\n\"a,b\",\"c\"\"d\"\n",
            "CSV escaping is incorrect."
        );
        requireTest(
            !writeRowsToCsvFile(path, {"first", "second"}, {{"one"}}),
            "A malformed row width was accepted."
        );
        std::filesystem::remove(path);
    }

    // Scenario: one start-equals-goal iterative run succeeds. Expected: one run directory contains exact stats/solution schemas and CLI metadata, with no conflicts file.
    {
        std::vector<std::vector<int>> freeCells {{1}};
        std::vector<mapf::Agent> agents {agent(42, 0, 0, 0, 0)};
        mapf::Instance instance(&freeCells, 1, 1, agents);
        std::list<mapf::Cell*> path {instance.getGrid().getCellPtr(0, 0)};
        mapf::experiments::ExperimentRunResult run {
            .metrics = mapf::Result(true, 0, 0, 0.0, 0.25),
            .initialPaths = {path},
            .solutionPaths = {path},
            .remainingConflicts = {},
            .numAgents = 1,
            .solver = "LocalPathRepairIterativeSolver",
            .continueIfFailed = true,
            .multithreading = false,
            .numThreads = 1,
            .localRepair = true
        };

        const std::set<std::filesystem::path> before = resultDirectories();
        requireTest(
            mapf::experiments::writeExperimentArtifacts(instance, run, 0.5),
            "Successful artifact bundle write failed."
        );
        const std::filesystem::path directory = onlyNewDirectory(
            before,
            resultDirectories()
        );
        const std::string prefix = directory.filename().string();
        const std::filesystem::path stats = directory / (prefix + "_stats.csv");
        const std::filesystem::path solution = directory / (prefix + "_solution.csv");
        const std::filesystem::path conflicts = directory / (prefix + "_conflicts.csv");

        requireTest(std::filesystem::exists(stats), "Stats artifact is missing.");
        requireTest(std::filesystem::exists(solution), "Solution artifact is missing.");
        requireTest(!std::filesystem::exists(conflicts), "Successful run wrote conflicts.");
        requireTest(
            readFile(stats).rfind(
                "map,instance_name,num_agents,success,paths_resolved,sumOfCosts,makespan,injustice,durationSeconds,time,multithreading,num_threads,solver,continue_if_failed\n",
                0
            ) == 0,
            "Stats header is incorrect."
        );
        requireTest(
            readFile(stats).find("\n-,-,1,true,") != std::string::npos,
            "Stats agent count is incorrect."
        );
        requireTest(
            readFile(stats).find(
                ",false,1,LocalPathRepairIterativeSolver,true\n"
            ) != std::string::npos,
            "Stats solver metadata is incorrect."
        );
        requireTest(
            readFile(solution).rfind(
                "agent_id,agent_scenario_bucket,start,goal,optimum_path,solution_path,optimum_path_cost,solution_path_cost,success_optimum_path,success_solution_path\n",
                0
            ) == 0,
            "Solution header is incorrect."
        );
        requireTest(
            readFile(solution).find(
                "42,-1,0-0,0-0,0-0,0-0,0,0,true,true"
            ) != std::string::npos,
            "Solution zero-cost serialization is incorrect."
        );

        std::filesystem::remove_all(directory);

        // Scenario: normalized metadata claims two agents for a one-agent instance. Expected: the writer rejects the inconsistent count without creating a bundle.
        run.numAgents = 2;
        const std::set<std::filesystem::path> beforeInvalidCount = resultDirectories();
        requireTest(
            !mapf::experiments::writeExperimentArtifacts(instance, run, 0.5),
            "Mismatched normalized agent count was accepted."
        );
        requireTest(
            resultDirectories() == beforeInvalidCount,
            "Mismatched normalized agent count created an artifact bundle."
        );
    }

    // Scenario: local repair fails without a geometric conflict because its only final path is missing. Expected: the run directory contains a header-only conflicts CSV.
    {
        std::vector<std::vector<int>> freeCells {{1}};
        std::vector<mapf::Agent> agents {agent(1, 0, 0, 0, 0)};
        mapf::Instance instance(&freeCells, 1, 1, agents);
        std::list<mapf::Cell*> initialPath {instance.getGrid().getCellPtr(0, 0)};
        mapf::experiments::ExperimentRunResult run {
            .metrics = mapf::Result(false, 0, 0, 0.0, 0.1),
            .initialPaths = {initialPath},
            .solutionPaths = {{}},
            .remainingConflicts = {},
            .numAgents = 1,
            .solver = "LocalPathRepairParallelSolver",
            .continueIfFailed = false,
            .multithreading = true,
            .numThreads = 2,
            .localRepair = true
        };

        const std::set<std::filesystem::path> before = resultDirectories();
        requireTest(
            mapf::experiments::writeExperimentArtifacts(instance, run, 0.2),
            "Failed-run artifact bundle write failed."
        );
        const std::filesystem::path directory = onlyNewDirectory(
            before,
            resultDirectories()
        );
        const std::string prefix = directory.filename().string();
        requireTest(
            readFile(directory / (prefix + "_conflicts.csv")) ==
                "cell_1,cell_2,timestep,conflict_type,agents\n",
            "Failed run did not write a header-only conflicts artifact."
        );
        std::filesystem::remove_all(directory);
    }

    return 0;
}
