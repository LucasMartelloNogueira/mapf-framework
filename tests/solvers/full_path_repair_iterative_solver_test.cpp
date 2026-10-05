#include "mapf/solvers/full_path_repair_iterative_solver.hpp"
#include "mapf/pathfinding/a_star.hpp"
#include "mapf/utils.hpp"
#include "local_path_repair_solver_common.hpp"
#include "test_support.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace {
    using namespace mapf;

    Agent agent(int id, int sx, int sy, int gx, int gy) {
        return {.id = id, .scenarioId = 3, .currentPosition = {sx, sy},
            .startPosition = {sx, sy}, .goalPosition = {gx, gy}};
    }

    void requireSameConflicts(const SolutionConflicts& actual, const SolutionConflicts& expected) {
        requireTest(actual.vertexEvents.size() == expected.vertexEvents.size(), "Vertex event count differs.");
        for (const auto& [key, event] : expected.vertexEvents) {
            requireTest(actual.vertexEvents.at(key).participants == event.participants, "Vertex participants differ.");
        }
        requireTest(actual.edgeEvents.size() == expected.edgeEvents.size(), "Edge event count differs.");
        for (const auto& [key, event] : expected.edgeEvents) {
            const auto& other = actual.edgeEvents.at(key);
            requireTest(other.forward == event.forward && other.reverse == event.reverse, "Edge participants differ.");
        }
        requireTest(actual.byAgent.size() == expected.byAgent.size(), "Conflict slots differ.");
        for (std::size_t i = 0; i < expected.byAgent.size(); ++i) {
            requireTest(actual.byAgent[i].size() == expected.byAgent[i].size(), "Agent conflict count differs.");
            for (std::size_t j = 0; j < expected.byAgent[i].size(); ++j) {
                requireTest(!conflictRecordComesBefore(actual.byAgent[i][j], expected.byAgent[i][j]) &&
                    !conflictRecordComesBefore(expected.byAgent[i][j], actual.byAgent[i][j]), "Agent conflict differs.");
            }
        }
    }

    void requireSameState(const PathReservationState& actual, const PathReservationState& expected) {
        requireTest(actual.vertex_agents == expected.vertex_agents, "Temporal owners differ.");
        requireTest(actual.goal_reservations.size() == expected.goal_reservations.size(), "Goal counts differ.");
        for (const auto& [cell, goal] : expected.goal_reservations) {
            const auto& other = actual.goal_reservations.at(cell);
            requireTest(other.agentId == goal.agentId && other.arrivalTime == goal.arrivalTime, "Goal ownership differs.");
        }
        requireTest(actual.safeIntervalTable.blockedEdgeArrivals == expected.safeIntervalTable.blockedEdgeArrivals,
            "Reverse-edge reservations differ.");
        const auto& safe = actual.safeIntervalTable.safeIntervalsByCell;
        const auto& reference = expected.safeIntervalTable.safeIntervalsByCell;
        requireTest(safe.size() == reference.size(), "Safe table sizes differ.");
        for (const auto& [cell, intervals] : reference) {
            const auto& other = safe.at(cell);
            requireTest(other.size() == intervals.size() &&
                std::equal(other.begin(), other.end(), intervals.begin(), [](const Interval& a, const Interval& b) {
                    return a.start == b.start && a.end == b.end;
                }), "Safe interval endpoints differ.");
        }
    }

    LocalPathRepairResult solveChecked(Instance& instance) {
        FullPathRepairIterativeSolver solver(instance);
        auto result = solver.solve();
        const auto& agents = instance.getAgents();
        auto& grid = instance.getGrid();
        requireTest(result.paths.size() == agents.size() && result.initialPaths.size() == agents.size() &&
            result.pathCosts.size() == agents.size(), "Result slots lost instance order.");
        int total = 0;
        int maximum = 0;
        bool complete = true;
        std::vector<double> delays;
        for (std::size_t i = 0; i < agents.size(); ++i) {
            Cell* start = grid.getCellPtr(agents[i].startPosition.x, agents[i].startPosition.y);
            Cell* goal = grid.getCellPtr(agents[i].goalPosition.x, agents[i].goalPosition.y);
            AStarSolver astar;
            requireTest(result.initialPaths[i] == astar.solve(grid, start, goal), "Initial A* path changed.");
            const auto& path = result.paths[i];
            const int cost = path.empty() ? 0 : static_cast<int>(path.size()) - 1;
            requireTest(result.pathCosts[i] == cost, "Per-agent action cost differs.");
            total += cost;
            maximum = std::max(maximum, cost);
            if (path.empty()) {
                complete = false;
                continue;
            }
            requireTest(path.front() == start && path.back() == goal, "Full path endpoints changed.");
            Cell* previous = nullptr;
            for (Cell* cell : path) {
                requireTest(cell != nullptr && cell->isFree, "Invalid path cell.");
                if (previous != nullptr && previous != cell) {
                    const auto neighbors = grid.getNeighbors(previous);
                    requireTest(std::find(neighbors.begin(), neighbors.end(), cell) != neighbors.end(), "Invalid move.");
                }
                previous = cell;
            }
            if (!result.initialPaths[i].empty()) {
                delays.push_back(cost - (static_cast<int>(result.initialPaths[i].size()) - 1));
            }
        }
        double mean = 0.0;
        for (double delay : delays) mean += delay;
        if (!delays.empty()) mean /= delays.size();
        double variance = 0.0;
        for (double delay : delays) variance += (delay - mean) * (delay - mean);
        if (!delays.empty()) variance /= delays.size();
        requireTest(result.metrics.sumOfCosts == total && result.metrics.makespan == maximum &&
            std::abs(result.metrics.injustice - std::sqrt(variance)) < 1e-12 && result.metrics.durationSeconds >= 0.0,
            "Final metrics differ from committed paths.");
        requireSameConflicts(result.initialConflicts, getCollision(result.initialPaths));
        requireSameConflicts(result.remainingConflicts, getCollision(result.paths));
        requireSameState(result.reservations,
            local_path_repair_detail::buildReservationState(grid, agents, result.paths));
        requireTest(result.metrics.success == (complete && result.remainingConflicts.empty()), "Invalid success flag.");

        const auto repeated = solver.solve();
        requireTest(repeated.initialPaths == result.initialPaths && repeated.paths == result.paths &&
            repeated.pathCosts == result.pathCosts && repeated.metrics.success == result.metrics.success,
            "Repeated solve changed paths or leaked state.");
        requireSameConflicts(repeated.remainingConflicts, result.remainingConflicts);
        requireSameState(repeated.reservations, result.reservations);
        return result;
    }
}

int main() {
    using namespace mapf;
    // No conflicts: empty, unit, following, and a legal four-agent cycle.
    for (const auto& agents : std::vector<std::vector<Agent>>{
        {}, {agent(9, 0, 0, 0, 0)}, {agent(9, 0, 0, 2, 0)},
        {agent(8, 1, 0, 2, 0), agent(2, 0, 0, 1, 0)},
        {agent(7, 0, 0, 1, 0), agent(3, 1, 0, 1, 1),
         agent(5, 1, 1, 0, 1), agent(1, 0, 1, 0, 0)}
    }) {
        std::vector<std::vector<int>> free(3, std::vector<int>(3, 1));
        Instance instance(&free, 3, 3, agents);
        const auto result = solveChecked(instance);
        requireTest(result.metrics.success && result.paths == result.initialPaths, "Conflict-free paths changed.");
    }

    // Independent initialization continues after a missing path.
    {
        std::vector<std::vector<int>> free{{1, 0, 1, 1}};
        Instance instance(&free, 1, 4, {agent(3, 0, 0, 2, 0), agent(9, 2, 0, 3, 0)});
        const auto result = solveChecked(instance);
        requireTest(!result.metrics.success && result.initialPaths[0].empty() && !result.initialPaths[1].empty() &&
            result.paths == result.initialPaths && result.remainingConflicts.empty(), "Missing-path diagnostics lost.");
    }

    // Remove BOTH paths first; slot 1 has priority and slot 0 must wait.
    for (const auto& ids : std::vector<std::pair<int, int>>{{42, 7}, {10, -7},
        {std::numeric_limits<int>::max(), std::numeric_limits<int>::min()}}) {
        std::vector<std::vector<int>> free(3, std::vector<int>(3, 1));
        Instance instance(&free, 3, 3, {agent(ids.first, 0, 1, 2, 1), agent(ids.second, 1, 0, 1, 2)});
        const auto result = solveChecked(instance);
        requireTest(result.metrics.success && result.initialConflicts.vertexEvents.size() == 1 &&
            result.pathCosts == std::vector<int>({3, 2}) && result.paths[1] == result.initialPaths[1],
            "Full-group removal or real-ID priority is incorrect.");
        requireTest(result.metrics.sumOfCosts == 5 && result.metrics.makespan == 3 &&
            result.metrics.injustice == 0.5, "Crossing metrics are incorrect.");
    }

    // All three participants of a vertex must be repaired.
    {
        std::vector<std::vector<int>> free(3, std::vector<int>(3, 1));
        Instance instance(&free, 3, 3,
            {agent(30, 0, 1, 2, 1), agent(10, 1, 0, 1, 2), agent(20, 2, 1, 0, 1)});
        const auto result = solveChecked(instance);
        requireTest(result.initialConflicts.vertexEvents.at({instance.getGrid().getCellPtr(1, 1), 1})
            .participants.size() == 3 && result.metrics.success, "Three-agent repair failed.");
    }

    // A parked owner has no byAgent record at the visitor's later arrival.
    // Replanning the visitor first requires removing that owner's permanent tail.
    {
        std::vector<std::vector<int>> free(3, std::vector<int>(4, 1));
        Instance instance(&free, 3, 4, {agent(20, 2, 0, 2, 1), agent(5, 0, 1, 3, 1)});
        const auto result = solveChecked(instance);
        requireTest(result.initialConflicts.byAgent[0].empty() && !result.initialConflicts.vertexEvents.empty(),
            "Fixture must involve a virtual parked owner.");
        requireTest(result.metrics.success && result.paths[1] == result.initialPaths[1] && result.pathCosts[0] == 3,
            "Parked owner was omitted or permanent-goal arrival was not delayed.");
    }

    // A repairable swap requires a detour, not a following-move prohibition.
    {
        std::vector<std::vector<int>> free(2, std::vector<int>(2, 1));
        Instance instance(&free, 2, 2, {agent(1, 0, 0, 1, 0), agent(2, 1, 0, 0, 0)});
        const auto result = solveChecked(instance);
        requireTest(result.initialConflicts.edgeEvents.size() == 1 && result.metrics.success &&
            result.pathCosts == std::vector<int>({1, 3}), "Edge swap repair failed.");
    }

    // Two forward movers share a swap with a lower-ID reverse mover. Its event
    // outranks the earlier forward-only vertex event, so all three form the group.
    {
        std::vector<std::vector<int>> free(4, std::vector<int>(4, 1));
        free[1][2] = 0;
        free[3][1] = 0;
        Instance instance(&free, 4, 4, {agent(20, 0, 2, 3, 2), agent(30, 1, 1, 2, 3), agent(1, 3, 2, 0, 2)});
        const auto result = solveChecked(instance);
        const auto edge = makeEdgeTime(instance.getGrid().getCellPtr(1, 2), instance.getGrid().getCellPtr(2, 2), 2);
        const auto& event = result.initialConflicts.edgeEvents.at(edge);
        requireTest(event.forward.size() == 2 && event.reverse.size() == 1 && result.metrics.success &&
            result.paths[2] == result.initialPaths[2], "Multi-participant swap or minimum-ID event order failed.");
    }

    // Overlapping events share a mover, but the first group's outsider remains
    // reserved. Replanning must resolve the mover's later collision as well.
    {
        std::vector<std::vector<int>> free(6, std::vector<int>(5, 1));
        Instance instance(&free, 6, 5, {agent(10, 0, 2, 4, 2), agent(1, 1, 1, 1, 4), agent(30, 3, 5, 3, 0)});
        const auto result = solveChecked(instance);
        requireTest(result.initialConflicts.vertexEvents.size() == 2 && result.metrics.success &&
            result.paths[2] == result.initialPaths[2] && result.pathCosts == std::vector<int>({5, 3, 5}),
            "A group's complete replacement ignored an outsider's future occupancy.");
    }

    // The first replacement succeeds but the second cannot pass in a two-cell corridor.
    {
        std::vector<std::vector<int>> free{{1, 1}};
        Instance instance(&free, 1, 2, {agent(0, 0, 0, 1, 0), agent(1, 1, 0, 0, 0)});
        const auto result = solveChecked(instance);
        requireTest(!result.metrics.success && result.paths == result.initialPaths, "Failed group was partly committed.");
        requireSameConflicts(result.remainingConflicts, result.initialConflicts);
    }

    // Disconnected groups: ID priority commits the crossing before failing the swap.
    for (bool failFirst : {false, true}) {
        std::vector<std::vector<int>> free{{0, 0, 0, 1, 1, 1}, {1, 1, 0, 1, 1, 1}, {0, 0, 0, 1, 1, 1}};
        const int swapId = failFirst ? 0 : 20;
        const int crossingId = failFirst ? 20 : 0;
        Instance instance(&free, 3, 6, {agent(swapId, 0, 1, 1, 1), agent(swapId + 1, 1, 1, 0, 1),
            agent(crossingId, 3, 1, 5, 1), agent(crossingId + 1, 4, 0, 4, 2)});
        const auto result = solveChecked(instance);
        requireTest(!result.metrics.success && result.paths[0] == result.initialPaths[0] &&
            result.paths[1] == result.initialPaths[1] && result.remainingConflicts.edgeEvents.size() == 1,
            "Failed group changed committed swap paths.");
        requireTest(result.remainingConflicts.vertexEvents.empty() == !failFirst,
            "Event priority or preservation of an earlier successful group failed.");
    }

    // Two successful groups and an uninvolved parked agent exercise fresh detection.
    {
        std::vector<std::vector<int>> free(3, std::vector<int>(8, 1));
        for (auto& row : free) row[3] = 0;
        Instance instance(&free, 3, 8, {agent(50, 4, 1, 6, 1), agent(51, 5, 0, 5, 2),
            agent(10, 0, 1, 2, 1), agent(11, 1, 0, 1, 2), agent(-9, 7, 2, 7, 2)});
        const auto result = solveChecked(instance);
        requireTest(result.metrics.success && result.initialConflicts.vertexEvents.size() == 2 &&
            result.paths[4] == result.initialPaths[4] && result.pathCosts == std::vector<int>({2, 3, 2, 3, 0}),
            "Multiple groups or outsider reservations failed.");
    }

    // Invalid instances are rejected before a solver can be constructed.
    for (bool duplicateStart : {false, true}) {
        std::vector<std::vector<int>> free(2, std::vector<int>(2, 1));
        bool rejected = false;
        try {
            Instance instance(&free, 2, 2, {agent(1, 0, 0, 1, 1),
                duplicateStart ? agent(2, 0, 0, 1, 0) : agent(2, 1, 0, 1, 1)});
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        requireTest(rejected, "Invalid instance accepted.");
    }
}
