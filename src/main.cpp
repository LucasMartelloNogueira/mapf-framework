#include "experiment_utils.hpp"

#include "mapf/core/instance.hpp"
#include "mapf/solvers/full_path_repair_iterative_solver.hpp"
#include "mapf/solvers/local_path_repair_iterative_solver.hpp"
#include "mapf/solvers/local_path_repair_parallel_solver.hpp"
#include "mapf/solvers/priority_planning_solver.hpp"

#include <charconv>
#include <chrono>
#include <exception>
#include <iostream>
#include <list>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
    struct CliOptions {
        std::string map;
        std::string scenario;
        std::string solver;
        int agents = 0;
        std::optional<std::size_t> threads;
        mapf::LocalRepairStrategy localRepairStrategy = mapf::LocalRepairStrategy::RESOLVE_BY_AGENT;
        bool localRepairStrategyProvided = false;
    };

    void printUsage(std::ostream& output, const char* binary) {
        output
            << "Usage: " << binary
            << " -map <map> -scen <scenario> -solver <solver> -agents <n>"
            << " [-threads <t>]"
            << " [-localRepairStrategy <RESOLVE_BY_AGENT|RESOLVE_BY_TIME>]\n"
            << "Local repair strategy applies to local solvers; default: RESOLVE_BY_AGENT.\n"
            << "Solvers: PriorityPlanningSolver, LocalPathRepairParallelSolver, "
            << "LocalPathRepairIterativeSolver, FullPathRepairIterativeSolver\n"
            << "FullPathRepairIterativeSolver uses fixed ID priority and accepts no optional flags.\n";
    }

    int parseInteger(const std::string& value, const std::string& flag) {
        int parsed = 0;
        const char* begin = value.data();
        const char* end = begin + value.size();
        const auto [position, error] = std::from_chars(begin, end, parsed);
        if (error != std::errc() || position != end || value.empty()) {
            throw std::invalid_argument(flag + " requires a valid integer.");
        }
        return parsed;
    }

    mapf::LocalRepairStrategy parseLocalRepairStrategy(const std::string& value) {
        if (value == "RESOLVE_BY_AGENT") {
            return mapf::LocalRepairStrategy::RESOLVE_BY_AGENT;
        }
        if (value == "RESOLVE_BY_TIME") {
            return mapf::LocalRepairStrategy::RESOLVE_BY_TIME;
        }
        throw std::invalid_argument(
            "-localRepairStrategy accepts only RESOLVE_BY_AGENT or RESOLVE_BY_TIME.");
    }

    CliOptions parseOptions(int argc, char* argv[]) {
        std::optional<std::string> map;
        std::optional<std::string> scenario;
        std::optional<std::string> solver;
        std::optional<std::string> agents;
        std::optional<std::string> threads;
        std::optional<std::string> localRepairStrategy;

        for (int index = 1; index < argc; index += 2) {
            const std::string flag = argv[index];
            if (index + 1 >= argc) {
                throw std::invalid_argument("Missing value for flag '" + flag + "'.");
            }

            const std::string value = argv[index + 1];
            std::optional<std::string>* target = nullptr;

            if (flag == "-map") {
                target = &map;
            } else if (flag == "-scen") {
                target = &scenario;
            } else if (flag == "-solver") {
                target = &solver;
            } else if (flag == "-agents") {
                target = &agents;
            } else if (flag == "-threads") {
                target = &threads;
            } else if (flag == "-localRepairStrategy") {
                target = &localRepairStrategy;
            } else {
                throw std::invalid_argument("Unknown flag or positional argument '" + flag + "'.");
            }

            if (target->has_value()) {
                throw std::invalid_argument("Duplicate flag '" + flag + "'.");
            }
            *target = value;
        }

        if (!map || !scenario || !solver || !agents) {
            throw std::invalid_argument(
                "Flags -map, -scen, -solver, and -agents are required."
            );
        }

        CliOptions options;
        options.map = *map;
        options.scenario = *scenario;
        options.solver = *solver;
        options.agents = parseInteger(*agents, "-agents");
        if (options.agents < 0) {
            throw std::invalid_argument("-agents must be non-negative.");
        }

        if (threads) {
            const int parsedThreads = parseInteger(*threads, "-threads");
            if (parsedThreads <= 0) {
                throw std::invalid_argument("-threads must be greater than zero.");
            }
            options.threads = static_cast<std::size_t>(parsedThreads);
        }

        if (localRepairStrategy) {
            options.localRepairStrategy = parseLocalRepairStrategy(*localRepairStrategy);
            options.localRepairStrategyProvided = true;
        }

        if (options.solver == "PriorityPlanningSolver") {
            if (options.localRepairStrategyProvided) {
                throw std::invalid_argument(
                    "-localRepairStrategy is not accepted by PriorityPlanningSolver.");
            }
            if (options.threads) {
                throw std::invalid_argument(
                    "-threads is not accepted by PriorityPlanningSolver."
                );
            }
        } else if (options.solver == "LocalPathRepairParallelSolver") {
            if (!options.threads) {
                throw std::invalid_argument(
                    "-threads is required by LocalPathRepairParallelSolver."
                );
            }
        } else if (options.solver == "LocalPathRepairIterativeSolver") {
            if (options.threads) {
                throw std::invalid_argument(
                    "-threads is not accepted by LocalPathRepairIterativeSolver."
                );
            }
        } else if (options.solver == "FullPathRepairIterativeSolver") {
            if (options.threads || options.localRepairStrategyProvided) {
                throw std::invalid_argument(
                    "FullPathRepairIterativeSolver does not accept -threads "
                    "or -localRepairStrategy.");
            }
        } else {
            throw std::invalid_argument("Unknown solver '" + options.solver + "'.");
        }

        return options;
    }

    std::list<int> instanceAgentIds(const mapf::Instance& instance) {
        std::list<int> agentIds;
        for (const mapf::Agent& agent : instance.getAgents()) {
            agentIds.push_back(agent.id);
        }
        return agentIds;
    }
}

int runCli(int argc, char* argv[]) {
    try {
        const CliOptions options = parseOptions(argc, argv);
        const auto experimentStartedAt = std::chrono::steady_clock::now();
        mapf::Instance instance(options.map, options.scenario, options.agents);
        mapf::experiments::ExperimentRunResult run;
        run.numAgents = options.agents;
        double experimentTimeSeconds = 0.0;

        if (options.solver == "PriorityPlanningSolver") {
            mapf::PriorityPlanningSolver solver(instance);
            run.metrics = solver.solve(instanceAgentIds(instance));
            experimentTimeSeconds =
                mapf::experiments::elapsedSeconds(experimentStartedAt);
            run.initialPaths = solver.getInitialPaths();
            run.solutionPaths = solver.getPaths();
            run.solver = "PriorityPlanningSolver";
            run.multithreading = false;
            run.numThreads = 1;
            run.localRepair = false;
        } else if (options.solver == "LocalPathRepairParallelSolver") {
            mapf::LocalPathRepairParallelSolver solver(
                instance,
                options.threads.value(),
                options.localRepairStrategy
            );
            mapf::LocalPathRepairResult result = solver.solve();
            experimentTimeSeconds =
                mapf::experiments::elapsedSeconds(experimentStartedAt);
            run.metrics = result.metrics;
            run.initialPaths = std::move(result.initialPaths);
            run.solutionPaths = std::move(result.paths);
            run.solver = "LocalPathRepairParallelSolver";
            run.multithreading = true;
            run.numThreads = options.threads.value();
            run.localRepair = true;
            run.localRepairStrategy = options.localRepairStrategy;
        } else if (options.solver == "LocalPathRepairIterativeSolver") {
            mapf::LocalPathRepairIterativeSolver solver(
                instance,
                options.localRepairStrategy
            );
            mapf::LocalPathRepairResult result = solver.solve();
            experimentTimeSeconds =
                mapf::experiments::elapsedSeconds(experimentStartedAt);
            run.metrics = result.metrics;
            run.initialPaths = std::move(result.initialPaths);
            run.solutionPaths = std::move(result.paths);
            run.solver = "LocalPathRepairIterativeSolver";
            run.multithreading = false;
            run.numThreads = 1;
            run.localRepair = true;
            run.localRepairStrategy = options.localRepairStrategy;
        } else if (options.solver == "FullPathRepairIterativeSolver") {
            mapf::FullPathRepairIterativeSolver solver(instance);
            mapf::LocalPathRepairResult result = solver.solve();
            experimentTimeSeconds =
                mapf::experiments::elapsedSeconds(experimentStartedAt);
            run.metrics = result.metrics;
            run.initialPaths = std::move(result.initialPaths);
            run.solutionPaths = std::move(result.paths);
            run.solver = "FullPathRepairIterativeSolver";
            run.multithreading = false;
            run.numThreads = 1;
            run.localRepair = false;
            run.localRepairStrategy = std::nullopt;
        } else {
            throw std::invalid_argument("Unknown solver '" + options.solver + "'.");
        }

        try {
            if (!mapf::experiments::writeExperimentArtifacts(
                instance,
                run,
                experimentTimeSeconds
            )) {
                std::cerr << "Error: could not write the experiment artifact bundle.\n";
                return 1;
            }
        } catch (const std::exception& error) {
            std::cerr << "Error: could not write experiment artifacts: "
                << error.what() << '\n';
            return 1;
        }

        return run.metrics.success ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        printUsage(std::cerr, argc > 0 ? argv[0] : "mapf_app");
        return 2;
    }
}

#ifndef MAPF_CLI_NO_MAIN
int main(int argc, char* argv[]) {
    return runCli(argc, argv);
}
#endif
