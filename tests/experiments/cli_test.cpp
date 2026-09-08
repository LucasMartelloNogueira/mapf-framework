#include "experiment_utils.hpp"
#include "test_support.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <string>
#include <utility>
#include <vector>

int runCli(int argc, char* argv[]);

namespace {
    int invoke(std::vector<std::string> arguments) {
        std::vector<char*> argv;
        argv.reserve(arguments.size());
        for (std::string& argument : arguments) {
            argv.push_back(argument.data());
        }
        return runCli(static_cast<int>(argv.size()), argv.data());
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

    std::filesystem::path findNewDirectory(
        const std::set<std::filesystem::path>& before
    ) {
        std::vector<std::filesystem::path> added;
        for (const std::filesystem::path& path : resultDirectories()) {
            if (!before.contains(path)) {
                added.push_back(path);
            }
        }
        requireTest(added.size() == 1, "CLI did not create exactly one run directory.");
        return added.front();
    }

    std::string readFile(const std::filesystem::path& path) {
        std::ifstream input(path);
        return std::string(
            std::istreambuf_iterator<char>(input),
            std::istreambuf_iterator<char>()
        );
    }

    void requireSuccessfulRun(
        const std::filesystem::path& mapPath,
        const std::filesystem::path& scenarioPath,
        const std::string& solver,
        bool continueIfFailed,
        bool parallel
    ) {
        std::vector<std::string> arguments {
            "mapf_app",
            "-agents",
            "1",
            "-solver",
            solver,
            "-scen",
            scenarioPath.string(),
            "-map",
            mapPath.string()
        };
        if (parallel) {
            arguments.push_back("-threads");
            arguments.push_back("2");
        }
        if (solver != "PriorityPlanningSolver") {
            arguments.push_back("-continue_if_failed");
            arguments.push_back(continueIfFailed ? "true" : "false");
        }

        const std::set<std::filesystem::path> before = resultDirectories();
        requireTest(invoke(std::move(arguments)) == 0, solver + " CLI run failed.");
        const std::filesystem::path directory = findNewDirectory(before);
        const std::string prefix = directory.filename().string();
        const std::string stats = readFile(
            directory / (prefix + "_stats.csv")
        );
        requireTest(
            stats.rfind(
                "map,instance_name,num_agents,success,paths_resolved,sumOfCosts,makespan,injustice,durationSeconds,time,multithreading,num_threads,solver,continue_if_failed\n",
                0
            ) == 0,
            "Stats header is incorrect."
        );
        requireTest(
            stats.find("\nempty-8-8.map,scenario file.scen,1,true,") !=
                std::string::npos,
            "Stats agent count did not match the -agents value."
        );
        requireTest(
            stats.find(
                "," + solver + "," +
                (continueIfFailed ? "true" : "false") + "\n"
            ) != std::string::npos,
            solver + " metadata was not serialized from CLI options."
        );
        std::filesystem::remove_all(directory);
    }
}

int main() {
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() /
        ("mapf cli test " + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()
        ));
    std::filesystem::create_directories(directory);
    const std::filesystem::path mapPath = directory / "empty-8-8.map";
    const std::filesystem::path scenarioPath = directory / "scenario file.scen";
    std::filesystem::copy_file(
        mapf::experiments::repositoryRoot() / "benchmarks" / "maps" / "empty-8-8.map",
        mapPath
    );
    {
        std::ofstream scenario(scenarioPath);
        scenario << "version 1\n11 empty-8-8.map 8 8 0 0 7 7 14\n";
    }

    // Scenario: every supported solver receives flags in a noncanonical order and paths whose parent contains spaces. Expected: explicit dispatch succeeds and stats metadata matches effective CLI options.
    requireSuccessfulRun(
        mapPath,
        scenarioPath,
        "PriorityPlanningSolver",
        false,
        false
    );
    requireSuccessfulRun(
        mapPath,
        scenarioPath,
        "LocalPathRepairParallelSolver",
        false,
        true
    );
    requireSuccessfulRun(
        mapPath,
        scenarioPath,
        "LocalPathRepairIterativeSolver",
        true,
        false
    );

    // Scenario: two agents swap an edge in a one-cell corridor. Expected: CLI returns algorithmic-failure code 1 and writes final paths plus the unresolved normalized conflict.
    {
        const std::filesystem::path corridorMap = directory / "corridor.map";
        const std::filesystem::path corridorScenario = directory / "corridor.scen";
        {
            std::ofstream map(corridorMap);
            map << "type octile\nheight 1\nwidth 2\nmap\n..\n";
        }
        {
            std::ofstream scenario(corridorScenario);
            scenario
                << "version 1\n"
                << "3 corridor.map 2 1 0 0 1 0 1\n"
                << "3 corridor.map 2 1 1 0 0 0 1\n";
        }

        const std::set<std::filesystem::path> before = resultDirectories();
        requireTest(
            invoke({
                "mapf_app", "-map", corridorMap.string(),
                "-scen", corridorScenario.string(),
                "-solver", "LocalPathRepairIterativeSolver",
                "-agents", "2", "-continue_if_failed", "true"
            }) == 1,
            "Algorithmic local-repair failure did not return code 1."
        );
        const std::filesystem::path runDirectory = findNewDirectory(before);
        const std::string prefix = runDirectory.filename().string();
        const std::string conflicts = readFile(
            runDirectory / (prefix + "_conflicts.csv")
        );
        const std::string solution = readFile(
            runDirectory / (prefix + "_solution.csv")
        );
        const std::string stats = readFile(
            runDirectory / (prefix + "_stats.csv")
        );
        requireTest(
            stats.find("\ncorridor.map,corridor.scen,2,false,") !=
                std::string::npos,
            "Failed CLI run did not serialize the -agents value."
        );
        requireTest(
            conflicts.find("0-0,1-0,1,edge,0|1") != std::string::npos,
            "Failed CLI run did not serialize its final edge conflict."
        );
        requireTest(
            solution.find(",true,false\n") != std::string::npos,
            "A conflicted final path was not marked unsuccessful."
        );
        std::filesystem::remove_all(runDirectory);
    }

    // Scenario: invalid flags and solver-specific combinations are supplied. Expected: parsing returns code 2 and creates no result directory.
    {
        const std::set<std::filesystem::path> before = resultDirectories();
        requireTest(
            invoke({
                "mapf_app", "-map", mapPath.string(), "-scen", scenarioPath.string(),
                "-solver", "UnknownSolver", "-agents", "1"
            }) == 2,
            "Unknown solver did not return code 2."
        );
        requireTest(
            invoke({
                "mapf_app", "-map", mapPath.string(), "-scen", scenarioPath.string(),
                "-solver", "LocalPathRepairParallelSolver", "-agents", "1"
            }) == 2,
            "Missing parallel thread count did not return code 2."
        );
        requireTest(
            invoke({
                "mapf_app", "-map", mapPath.string(), "-scen", scenarioPath.string(),
                "-solver", "PriorityPlanningSolver", "-agents", "1",
                "-continue_if_failed", "false"
            }) == 2,
            "Priority continuation flag did not return code 2."
        );
        requireTest(
            invoke({
                "mapf_app", "-map", mapPath.string(), "-scen", scenarioPath.string(),
                "-solver", "LocalPathRepairIterativeSolver", "-agents", "1",
                "-continue_if_failed", "yes"
            }) == 2,
            "Malformed boolean did not return code 2."
        );
        requireTest(
            invoke({
                "mapf_app", "-map", (directory / "missing.map").string(),
                "-scen", scenarioPath.string(), "-solver", "PriorityPlanningSolver",
                "-agents", "1"
            }) == 2,
            "Missing input file did not return code 2."
        );
        requireTest(
            resultDirectories() == before,
            "Invalid CLI input created an artifact directory."
        );
    }

    std::filesystem::remove_all(directory);
    return 0;
}
