#include "mapf/pathfinding/a_star.hpp"
#include "mapf/solvers/local_path_repair_iterative_solver.hpp"
#include "mapf/solvers/local_path_repair_parallel_solver.hpp"
#include "mapf/utils.hpp"
#include "test_support.hpp"

#include <list>
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

    std::string pathCoordinates(const std::list<mapf::Cell*>& path) {
        std::string coordinates;
        for (mapf::Cell* cell : path) {
            coordinates += std::to_string(cell->position.x) + ",";
            coordinates += std::to_string(cell->position.y) + ";";
        }
        return coordinates;
    }

    void requireSamePaths(
        const std::vector<std::list<mapf::Cell*>>& first,
        const std::vector<std::list<mapf::Cell*>>& second,
        const std::string& message
    ) {
        requireTest(first.size() == second.size(), message + " (size)");
        for (std::size_t i = 0; i < first.size(); i++) {
            requireTest(
                pathCoordinates(first[i]) == pathCoordinates(second[i]),
                message + " (agent " + std::to_string(i) + ")"
            );
        }
    }
}

int main() {
    // Scenario: two shortest paths cross under either strategy. Expected: both adapters preserve identical initial paths and return the same repaired solution.
    for (auto strategy : {mapf::LocalRepairStrategy::RESOLVE_BY_AGENT, mapf::LocalRepairStrategy::RESOLVE_BY_TIME}) {
        std::vector<std::vector<int>> freeCells(3, std::vector<int>(3, 1));
        std::vector<mapf::Agent> agents {
            agent(10, 0, 1, 2, 1),
            agent(20, 1, 0, 1, 2)
        };
        mapf::Instance instance(&freeCells, 3, 3, agents);
        mapf::LocalPathRepairParallelSolver parallel(instance, 2, false, strategy);
        mapf::LocalPathRepairIterativeSolver iterative(instance, false, strategy);
        mapf::LocalPathRepairResult parallelResult = parallel.solve();
        mapf::LocalPathRepairResult iterativeResult = iterative.solve();

        requireTest(iterativeResult.metrics.success, "Iterative crossing repair failed.");
        requireTest(parallelResult.metrics.success, "Parallel crossing repair failed.");
        for (const mapf::Result& metrics : {iterativeResult.metrics, parallelResult.metrics}) {
            requireTest(metrics.numInitialConflicts == 1 && metrics.numResolvedConflicts == 1 &&
                metrics.numUnresolvedConflicts == 0, "Crossing conflict metrics are incorrect.");
        }
        requireSamePaths(
            iterativeResult.initialPaths,
            parallelResult.initialPaths,
            "Initial paths differ between adapters."
        );
        requireSamePaths(
            iterativeResult.paths,
            parallelResult.paths,
            "Repaired paths differ between adapters."
        );
        requireTest(
            !getCollision(iterativeResult.initialPaths).vertexEvents.empty(),
            "The preserved initial paths lost their crossing conflict."
        );
        requireTest(
            validateSolution(iterativeResult.paths),
            "The iterative repaired paths are invalid."
        );
    }

    // Scenario: one initial path is unreachable while a later agent is reachable. Expected: stop immediately in either continuation mode and keep diagnostic slots aligned.
    for (bool continueIfFailed : {false, true}) {
        std::vector<std::vector<int>> freeCells {{1, 0, 1, 1}};
        std::vector<mapf::Agent> agents {
            agent(3, 0, 0, 2, 0),
            agent(9, 2, 0, 3, 0)
        };
        mapf::Instance instance(&freeCells, 1, 4, agents);
        mapf::LocalPathRepairIterativeSolver solver(instance, continueIfFailed);
        mapf::LocalPathRepairResult result = solver.solve();

        requireTest(!result.metrics.success, "An unreachable initial path succeeded.");
        requireTest(result.metrics.numInitialConflicts == 0 && result.metrics.numResolvedConflicts == 0 &&
            result.metrics.numUnresolvedConflicts == 0, "Missing initial path reported conflicts.");
        requireTest(result.initialPaths.size() == 2, "Initial paths are not aligned.");
        requireTest(result.initialPaths[0].empty(), "The unreachable path is not empty.");
        requireTest(result.initialPaths[1].empty(), "The later reachable path was attempted after failure.");
        requireSamePaths(
            result.initialPaths,
            result.paths,
            "Initial-path failure changed committed paths."
        );
    }

    // Scenario: an edge swap in a one-cell-wide corridor cannot be repaired. Expected: continuation mode terminates with unchanged paths and the final edge conflict.
    {
        std::vector<std::vector<int>> freeCells(1, std::vector<int>(2, 1));
        std::vector<mapf::Agent> agents {
            agent(0, 0, 0, 1, 0),
            agent(1, 1, 0, 0, 0)
        };
        mapf::Instance instance(&freeCells, 1, 2, agents);
        mapf::LocalPathRepairIterativeSolver solver(instance, true);
        mapf::LocalPathRepairResult result = solver.solve();

        requireTest(!result.metrics.success, "Impossible continuation repair succeeded.");
        requireTest(result.metrics.numInitialConflicts == 1 && result.metrics.numResolvedConflicts == 0 &&
            result.metrics.numUnresolvedConflicts == 1, "Failed repair conflict metrics are incorrect.");
        requireSamePaths(
            result.initialPaths,
            result.paths,
            "Failed continuation changed committed paths."
        );
        requireTest(
            !result.remainingConflicts.edgeEvents.empty(),
            "The unresolved edge conflict was not recomputed at return."
        );
    }

    // Scenario: an isolated unrepairable edge swap precedes a repairable crossing in another component. Expected: continuation skips the swap, repairs the later crossing, and reports only the edge conflict that actually remains.
    {
        std::vector<std::vector<int>> freeCells {
            {0, 0, 0, 1, 1, 1},
            {1, 1, 0, 1, 1, 1},
            {0, 0, 0, 1, 1, 1}
        };
        std::vector<mapf::Agent> agents {
            agent(0, 0, 1, 1, 1),
            agent(1, 1, 1, 0, 1),
            agent(2, 3, 1, 5, 1),
            agent(3, 4, 0, 4, 2)
        };
        mapf::Instance instance(&freeCells, 3, 6, agents);
        mapf::LocalPathRepairIterativeSolver stopSolver(instance, false);
        mapf::LocalPathRepairIterativeSolver continueSolver(instance, true);
        mapf::LocalPathRepairResult stopped = stopSolver.solve();
        mapf::LocalPathRepairResult continued = continueSolver.solve();

        requireTest(!stopped.metrics.success, "The stopped mixed fixture succeeded.");
        requireTest(!continued.metrics.success, "The continued mixed fixture succeeded.");
        requireTest(stopped.metrics.numInitialConflicts == 2 && stopped.metrics.numResolvedConflicts == 0 &&
            stopped.metrics.numUnresolvedConflicts == 2, "Stopped repair conflict metrics are incorrect.");
        requireTest(continued.metrics.numInitialConflicts == 2 && continued.metrics.numResolvedConflicts == 1 &&
            continued.metrics.numUnresolvedConflicts == 1, "Partial repair conflict metrics are incorrect.");
        requireTest(
            !stopped.remainingConflicts.vertexEvents.empty(),
            "Immediate failure unexpectedly repaired the later crossing."
        );
        requireTest(
            continued.remainingConflicts.vertexEvents.empty(),
            "Continuation did not repair the later crossing."
        );
        requireTest(
            !continued.remainingConflicts.edgeEvents.empty(),
            "Continuation lost the genuinely unresolved edge swap."
        );
        requireTest(
            pathCoordinates(continued.paths[2]) != pathCoordinates(continued.initialPaths[2]) ||
                pathCoordinates(continued.paths[3]) != pathCoordinates(continued.initialPaths[3]),
            "Continuation did not commit a repair in the later component."
        );
    }

    // Scenario: the instance contains no agents. Expected: iterative repair returns a successful empty result with aligned empty vectors.
    {
        std::vector<std::vector<int>> freeCells {{1}};
        mapf::Instance instance(&freeCells, 1, 1, {});
        mapf::LocalPathRepairIterativeSolver solver(instance);
        mapf::LocalPathRepairResult result = solver.solve();

        requireTest(result.metrics.success, "The empty iterative instance failed.");
        requireTest(result.metrics.numInitialConflicts == 0 && result.metrics.numResolvedConflicts == 0 &&
            result.metrics.numUnresolvedConflicts == 0, "Empty instance reported conflicts.");
        requireTest(
            result.initialPaths.empty() && result.paths.empty() && result.pathCosts.empty(),
            "The empty iterative result contains path entries."
        );
    }

    return 0;
}
