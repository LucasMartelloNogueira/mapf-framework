#include "mapf/solvers/local_path_repair_parallel_solver.hpp"

#include "mapf/pathfinding/sipp/edge_key.hpp"
#include "mapf/utils.hpp"
#include "test_support.hpp"

#include <list>
#include <stdexcept>
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
        std::string value;
        for (mapf::Cell* cell : path) {
            value += std::to_string(cell->position.x);
            value += ',';
            value += std::to_string(cell->position.y);
            value += ';';
        }
        return value;
    }

    bool hasCellConflictAt(const mapf::SolutionConflicts& conflicts, int time) {
        for (const mapf::CellConflict& conflict : conflicts.cellConflicts) {
            if (conflict.time == time) {
                return true;
            }
        }

        return false;
    }

    struct CrossingSnapshot {
        bool success;
        std::vector<std::string> paths;
    };

    CrossingSnapshot solveCrossingSnapshot(std::size_t threadCount) {
        std::vector<std::vector<int>> freeCells(3, std::vector<int>(3, 1));
        std::vector<mapf::Agent> agents {
            agent(10, 0, 1, 2, 1),
            agent(20, 1, 0, 1, 2)
        };
        mapf::Instance instance(&freeCells, 3, 3, agents);
        mapf::LocalPathRepairParallelSolver solver(instance, threadCount);
        mapf::LocalPathRepairResult result = solver.solve();

        CrossingSnapshot snapshot {.success = result.metrics.success, .paths = {}};
        for (const std::list<mapf::Cell*>& path : result.paths) {
            snapshot.paths.push_back(pathCoordinates(path));
        }

        return snapshot;
    }
}

int main() {
    // Scenario: two unconstrained shortest paths cross at their center and produce delayed suffix Scenario 2.4. Expected: sequential repair returns a valid solution.
    {
        std::vector<std::vector<int>> freeCells(3, std::vector<int>(3, 1));
        std::vector<mapf::Agent> agents {
            agent(10, 0, 1, 2, 1),
            agent(20, 1, 0, 1, 2)
        };
        mapf::Instance instance(&freeCells, 3, 3, agents);
        mapf::LocalPathRepairParallelSolver solver(instance, 2);
        mapf::LocalPathRepairResult result = solver.solve();

        requireTest(!result.initialConflicts.cellConflicts.empty(), "The crossing fixture did not create an initial conflict.");
        requireTest(result.metrics.success, "The crossing paths were not repaired.");
        requireTest(result.remainingConflicts.cellConflicts.empty(), "A vertex conflict remained after crossing repair.");
        requireTest(result.remainingConflicts.edgeConflicts.empty(), "An edge conflict remained after crossing repair.");
        requireTest(validateSolution(result.paths), "The repaired crossing solution is invalid.");
    }

    // Scenario: the same parallel input uses one, two, and more workers than agents. Expected: path ownership and final coordinates remain deterministic.
    {
        CrossingSnapshot oneWorker = solveCrossingSnapshot(1);
        CrossingSnapshot twoWorkers = solveCrossingSnapshot(2);
        CrossingSnapshot manyWorkers = solveCrossingSnapshot(8);

        requireTest(oneWorker.success && twoWorkers.success && manyWorkers.success, "A supported worker count failed.");
        requireTest(oneWorker.paths.size() == 2, "Paths are not aligned with the two instance agents.");
        requireTest(oneWorker.paths[0] == twoWorkers.paths[0], "Agent zero changed path with worker count.");
        requireTest(oneWorker.paths[1] == twoWorkers.paths[1], "Agent one changed path with worker count.");
        requireTest(oneWorker.paths[0] == manyWorkers.paths[0], "Excess workers changed the deterministic result.");
    }

    // Scenario: an equal-time alternate bridge exists around a permanently occupied conflict cell (Scenario 1). Expected: repair preserves the original path cost.
    {
        std::vector<std::vector<int>> freeCells(3, std::vector<int>(2, 1));
        std::vector<mapf::Agent> agents {
            agent(0, 0, 0, 1, 1),
            agent(1, 0, 2, 0, 1)
        };
        mapf::Instance instance(&freeCells, 3, 2, agents);
        mapf::Cell* conflictCell = instance.getGrid().getCellPtr(0, 1);
        mapf::LocalPathRepairParallelSolver solver(instance, 2);
        mapf::LocalPathRepairResult result = solver.solve();

        requireTest(!result.initialConflicts.cellConflicts.empty(), "The Scenario 1 fixture did not create an initial conflict.");
        requireTest(result.metrics.success, "Equal-time suffix reconnection failed.");
        requireTest(result.pathCosts[0] == 2, "Scenario 1 did not preserve the original path cost.");
        requireTest(validateSolution(result.paths), "Scenario 1 produced an invalid solution.");
        requireTest(!result.reservations.vertex_agents.at(conflictCell).contains(0), "The replaced path left stale vertex membership.");
    }

    // Scenario: a delayed bridge reaches a suffix goal after another agent leaves it (Scenario 2.3). Expected: the real goal is accepted only in its infinite safe interval.
    {
        std::vector<std::vector<int>> freeCells(3, std::vector<int>(3, 1));
        std::vector<mapf::Agent> agents {
            agent(0, 0, 1, 2, 1),
            agent(1, 1, 0, 2, 2)
        };
        mapf::Instance instance(&freeCells, 3, 3, agents);
        mapf::LocalPathRepairParallelSolver solver(instance, 2);
        mapf::LocalPathRepairResult result = solver.solve();

        requireTest(result.metrics.success, "Scenario 2.3 delayed-goal repair failed.");
        requireTest(result.pathCosts[0] >= 3, "Scenario 2.3 accepted the old unsafe goal time.");
        requireTest(validateSolution(result.paths), "Scenario 2.3 produced an invalid solution.");
    }

    // Scenario: two agents swap one edge while each targets the other's start. Expected: repair detours and removes the obsolete reverse-edge reservation.
    {
        std::vector<std::vector<int>> freeCells(2, std::vector<int>(2, 1));
        std::vector<mapf::Agent> agents {
            agent(7, 0, 0, 1, 0),
            agent(8, 1, 0, 0, 0)
        };
        mapf::Instance instance(&freeCells, 2, 2, agents);
        mapf::Cell* oldStart = instance.getGrid().getCellPtr(0, 0);
        mapf::Cell* oldGoal = instance.getGrid().getCellPtr(1, 0);
        mapf::LocalPathRepairParallelSolver solver(instance, 2);
        mapf::LocalPathRepairResult result = solver.solve();

        requireTest(result.metrics.success, "The edge swap was not repaired.");
        requireTest(result.initialConflicts.edgeConflicts.size() == 1, "The edge-swap fixture did not report its initial conflict.");
        const auto obsoleteEdge = result.reservations.safeIntervalTable.blockedEdgeArrivals.find({oldGoal, oldStart});
        requireTest(
            obsoleteEdge == result.reservations.safeIntervalTable.blockedEdgeArrivals.end() ||
                !obsoleteEdge->second.contains(1),
            "The repaired agent's obsolete edge reservation remained committed."
        );
        requireTest(result.reservations.goal_reservations.at(oldGoal) == result.pathCosts[0], "The repaired goal arrival was not refreshed.");
    }

    // Scenario: a longer path enters a shorter agent's permanently occupied goal. Expected: the moving agent routes around the stay-at-target reservation.
    {
        std::vector<std::vector<int>> freeCells(2, std::vector<int>(4, 1));
        std::vector<mapf::Agent> agents {
            agent(0, 1, 0, 1, 1),
            agent(1, 3, 1, 0, 1)
        };
        mapf::Instance instance(&freeCells, 2, 4, agents);
        mapf::LocalPathRepairParallelSolver solver(instance, 2);
        mapf::LocalPathRepairResult result = solver.solve();

        requireTest(result.metrics.success, "The stay-at-target conflict was not repaired.");
        requireTest(validateSolution(result.paths), "Stay-at-target repair returned a conflicting solution.");
        requireTest(result.reservations.vertex_agents.at(instance.getGrid().getCellPtr(1, 1)).contains(0), "Goal vertex membership lost the owning agent.");
    }

    // Scenario: delayed suffix repair sees no/no, no/potential, potential/no, and potential/potential vertex pairs. Expected: all four Scenario 2.3 transition cases produce one valid path.
    {
        std::vector<std::vector<int>> freeCells(3, std::vector<int>(7, 1));
        freeCells[2][3] = 0;
        std::vector<mapf::Agent> agents {
            agent(0, 0, 1, 6, 1),
            agent(1, 1, 0, 1, 2),
            agent(2, 3, 0, 4, 2)
        };
        mapf::Instance instance(&freeCells, 3, 7, agents);
        mapf::LocalPathRepairParallelSolver solver(instance, 3);
        mapf::LocalPathRepairResult result = solver.solve();

        requireTest(!result.initialConflicts.cellConflicts.empty(), "The Scenario 2.3 transition fixture did not create its first conflict.");
        requireTest(result.metrics.success, "The Scenario 2.3 transition cases were not repaired.");
        requireTest(result.pathCosts[0] > 6, "The Scenario 2.3 fixture did not delay the suffix.");
        requireTest(validateSolution(result.paths), "Scenario 2.3 transition repair returned an invalid solution.");
    }

    // Scenario: constructing the solver with zero workers. Expected: the constructor rejects the invalid thread count.
    {
        std::vector<std::vector<int>> freeCells(1, std::vector<int>(1, 1));
        mapf::Instance instance(&freeCells, 1, 1, {});
        bool threw = false;
        try {
            mapf::LocalPathRepairParallelSolver solver(instance, 0);
        } catch (const std::invalid_argument&) {
            threw = true;
        }

        requireTest(threw, "Zero workers were not rejected.");
    }

    // Scenario: an instance contains no agents. Expected: the solver returns an empty successful solution with zero metrics.
    {
        std::vector<std::vector<int>> freeCells(1, std::vector<int>(1, 1));
        mapf::Instance instance(&freeCells, 1, 1, {});
        mapf::LocalPathRepairParallelSolver solver(instance, 1);
        mapf::LocalPathRepairResult result = solver.solve();

        requireTest(result.metrics.success, "The empty instance should succeed.");
        requireTest(result.paths.empty() && result.pathCosts.empty(), "The empty instance returned path entries.");
        requireTest(result.metrics.sumOfCosts == 0 && result.metrics.makespan == 0, "The empty instance has nonzero metrics.");
    }

    // Scenario: two agents have the same fixed start. Expected: solving fails without attempting to mutate the endpoints.
    {
        std::vector<std::vector<int>> freeCells(2, std::vector<int>(2, 1));
        std::vector<mapf::Agent> agents {
            agent(0, 0, 0, 1, 0),
            agent(1, 0, 0, 0, 1)
        };
        mapf::Instance instance(&freeCells, 2, 2, agents);
        mapf::LocalPathRepairParallelSolver solver(instance, 2);
        mapf::LocalPathRepairResult result = solver.solve();

        requireTest(!result.metrics.success, "A shared start was incorrectly repaired.");
        requireTest(hasCellConflictAt(result.initialConflicts, 0), "The shared-start conflict was not preserved diagnostically.");
    }

    // Scenario: two agents have the same permanent destination. Expected: solving fails before local repair overwrites the goal reservation.
    {
        std::vector<std::vector<int>> freeCells(2, std::vector<int>(2, 1));
        std::vector<mapf::Agent> agents {
            agent(0, 0, 0, 1, 1),
            agent(1, 1, 0, 1, 1)
        };
        mapf::Instance instance(&freeCells, 2, 2, agents);
        mapf::LocalPathRepairParallelSolver solver(instance, 2);
        mapf::LocalPathRepairResult result = solver.solve();

        requireTest(!result.metrics.success, "Duplicate permanent goals were incorrectly accepted.");
    }

    // Scenario: an obstacle separates a valid start and goal. Expected: the empty initial A* path produces a diagnostic failure.
    {
        std::vector<std::vector<int>> freeCells {{1, 0, 1}};
        std::vector<mapf::Agent> agents {agent(0, 0, 0, 2, 0)};
        mapf::Instance instance(&freeCells, 1, 3, agents);
        mapf::LocalPathRepairParallelSolver solver(instance, 1);
        mapf::LocalPathRepairResult result = solver.solve();

        requireTest(!result.metrics.success, "An unreachable initial path was accepted.");
        requireTest(result.paths.size() == 1 && result.paths[0].empty(), "The unreachable path was not preserved diagnostically.");
    }

    // Scenario: an edge swap occurs in a one-cell-wide corridor with no waiting or detour available. Expected: local repair and complete fallback fail without changing the initial paths.
    {
        std::vector<std::vector<int>> freeCells(1, std::vector<int>(2, 1));
        std::vector<mapf::Agent> agents {
            agent(0, 0, 0, 1, 0),
            agent(1, 1, 0, 0, 0)
        };
        mapf::Instance instance(&freeCells, 1, 2, agents);
        mapf::LocalPathRepairParallelSolver solver(instance, 2);
        mapf::LocalPathRepairResult result = solver.solve();

        requireTest(!result.metrics.success, "An impossible local and full repair was accepted.");
        requireTest(result.paths[0].size() == 2 && result.paths[1].size() == 2, "Failed repair partially changed the committed paths.");
        requireTest(!result.remainingConflicts.edgeConflicts.empty(), "Failed repair lost its diagnostic edge conflict.");
    }

    // Scenario: one agent starts at its own goal. Expected: the solver returns a successful zero-cost one-cell path and permanent reservation.
    {
        std::vector<std::vector<int>> freeCells(1, std::vector<int>(1, 1));
        std::vector<mapf::Agent> agents {agent(42, 0, 0, 0, 0)};
        mapf::Instance instance(&freeCells, 1, 1, agents);
        mapf::LocalPathRepairParallelSolver solver(instance, 1);
        mapf::LocalPathRepairResult result = solver.solve();

        requireTest(result.metrics.success, "Start-equals-goal failed.");
        requireTest(result.paths[0].size() == 1 && result.pathCosts[0] == 0, "Start-equals-goal has the wrong path cost.");
        requireTest(result.reservations.goal_reservations.at(result.paths[0].back()) == 0, "Start-equals-goal reservation has the wrong arrival time.");
    }

    return 0;
}
