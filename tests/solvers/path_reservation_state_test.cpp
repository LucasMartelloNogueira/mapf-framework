#include "local_path_repair_solver_common.hpp"
#include "solver_utils.hpp"
#include "mapf/pathfinding/a_star_sipp.hpp"
#include "mapf/solvers/local_path_repair_iterative_solver.hpp"
#include "mapf/solvers/local_path_repair_parallel_solver.hpp"
#include "mapf/utils.hpp"
#include "test_support.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <initializer_list>
#include <new>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace allocation_test {
    thread_local int remaining = -1;
}

void* operator new(std::size_t size) {
    if (allocation_test::remaining == 0) throw std::bad_alloc();
    if (allocation_test::remaining > 0) --allocation_test::remaining;
    if (void* memory = std::malloc(size == 0 ? 1 : size)) return memory;
    throw std::bad_alloc();
}
void* operator new[](std::size_t size) { return ::operator new(size); }
void operator delete(void* memory) noexcept { std::free(memory); }
void operator delete[](void* memory) noexcept { std::free(memory); }
void operator delete(void* memory, std::size_t) noexcept { std::free(memory); }
void operator delete[](void* memory, std::size_t) noexcept { std::free(memory); }

namespace {
    using namespace mapf;
    using namespace mapf::solver_utils;
    using Paths = std::vector<std::list<Cell*>>;

    Agent agent(int id, Cell* start, Cell* goal) {
        return {.id = id, .currentPosition = start->position,
            .startPosition = start->position, .goalPosition = goal->position};
    }

    bool sameIntervals(const std::vector<Interval>& first, const std::vector<Interval>& second) {
        return first.size() == second.size() && std::equal(first.begin(), first.end(), second.begin(),
            [](const Interval& a, const Interval& b) { return a.start == b.start && a.end == b.end; });
    }

    void requireIntervals(const PathReservationState& state, Cell* cell,
        std::initializer_list<Interval> expected) {
        const auto found = state.safeIntervalTable.safeIntervalsByCell.find(cell);
        const std::vector<Interval> unrestricted {{0, SAFE_INTERVAL_INFINITY}};
        requireTest(sameIntervals(found == state.safeIntervalTable.safeIntervalsByCell.end()
            ? unrestricted : found->second, expected),
            "Unexpected safe interval endpoints.");
    }

    void requireSameState(const PathReservationState& actual, const PathReservationState& expected) {
        requireTest(actual.vertex_agents == expected.vertex_agents, "Explicit temporal owners differ.");
        requireTest(actual.goal_reservations.size() == expected.goal_reservations.size(), "Goal count differs.");
        for (const auto& [cell, goal] : expected.goal_reservations) {
            const auto& value = actual.goal_reservations.at(cell);
            requireTest(value.agentId == goal.agentId && value.arrivalTime == goal.arrivalTime,
                "Goal owner or arrival differs.");
        }
        const auto& table = actual.safeIntervalTable;
        const auto& reference = expected.safeIntervalTable;
        requireTest(table.blockedEdgeArrivals == reference.blockedEdgeArrivals, "Edge reservations differ.");
        requireTest(table.safeIntervalsByCell.size() == reference.safeIntervalsByCell.size(), "Safe cell count differs.");
        for (const auto& [cell, intervals] : reference.safeIntervalsByCell) {
            requireTest(sameIntervals(table.safeIntervalsByCell.at(cell), intervals), "Safe intervals differ.");
        }
    }

    template<class Exception, class Function>
    void requireThrows(Function function) {
        bool threw = false;
        try {
            function();
        } catch (const Exception&) {
            threw = true;
        }
        requireTest(threw, "Expected reservation contract error was not reported.");
    }

    // Independent bounded time-space oracle: directly follow each path and park
    // at its last cell, without using the builder or interval merging routines.
    void requireOccupancyMatchesPaths(Grid& grid, const PathReservationState& state, const Paths& paths) {
        std::vector<std::vector<Cell*>> indexed;
        std::size_t horizon = 0;
        for (const auto& path : paths) {
            indexed.emplace_back(path.begin(), path.end());
            horizon = std::max(horizon, path.size());
        }
        for (Cell& cell : grid.getCells()) {
            const auto found = state.safeIntervalTable.safeIntervalsByCell.find(&cell);
            const std::vector<Interval> unrestricted {{0, SAFE_INTERVAL_INFINITY}};
            const auto& intervals = found == state.safeIntervalTable.safeIntervalsByCell.end()
                ? unrestricted : found->second;
            for (std::size_t t = 0; t <= horizon + 1; ++t) {
                bool occupied = false;
                for (const auto& path : indexed) {
                    if (!path.empty() && path[std::min(t, path.size() - 1)] == &cell) {
                        occupied = true;
                    }
                }
                const bool safe = std::any_of(intervals.begin(), intervals.end(), [t](const Interval& interval) {
                    return interval.start <= static_cast<int>(t) && static_cast<int>(t) <= interval.end;
                });
                requireTest(safe != occupied, "Time-space oracle disagrees with the safe table.");
            }
        }
        for (const auto& [edge, arrivals] : state.safeIntervalTable.blockedEdgeArrivals) {
            requireTest(!arrivals.empty() && edge.from != edge.to, "Empty edge entry or wait edge remains.");
            for (int t : arrivals) {
                requireTest(std::any_of(indexed.begin(), indexed.end(), [&](const auto& path) {
                    return t > 0 && static_cast<std::size_t>(t) < path.size() &&
                        path[t - 1] == edge.to && path[t] == edge.from;
                }), "A blocked edge has no actual contributing move.");
            }
        }
        for (const auto& path : indexed) {
            for (std::size_t t = 1; t < path.size(); ++t) {
                if (path[t - 1] != path[t]) {
                    requireTest(state.safeIntervalTable.blockedEdgeArrivals.at({path[t], path[t - 1]})
                        .contains(static_cast<int>(t)), "An actual move lost its reverse-edge reservation.");
                }
            }
        }
    }

    std::list<Cell*> randomPath(Grid& grid, Cell* start, Cell* goal, std::mt19937& random) {
        std::list<Cell*> path {start};
        const int steps = static_cast<int>(random() % 18);
        for (int i = 0; i < steps; ++i) {
            const auto neighbors = grid.getNeighbors(path.back());
            std::vector<Cell*> choices(neighbors.begin(), neighbors.end());
            choices.push_back(path.back()); // Explicit waits, including revisits.
            path.push_back(choices[random() % choices.size()]);
        }
        while (path.back() != goal) {
            auto p = path.back()->position;
            if (p.x != goal->position.x) {
                p.x += p.x < goal->position.x ? 1 : -1;
            } else {
                p.y += p.y < goal->position.y ? 1 : -1;
            }
            path.push_back(grid.getCellPtr(p.x, p.y));
        }
        return path;
    }
}

int main() {
    using namespace mapf;
    using namespace mapf::solver_utils;
    using mapf::local_path_repair_detail::repairInitialPaths;
    constexpr int infinity = SAFE_INTERVAL_INFINITY;

    // Count cell/time events, not participants; ignore self and missing/empty
    // entries. Waiting visits still advance time. The query must not allocate.
    {
        Grid grid(1, 5);
        auto cell = [&](int x) { return grid.getCellPtr(x, 0); };
        const VertexOccupants occupants {
            {cell(0), {{0, {10}}}},
            {cell(1), {{1, {10, 42, 99}}, {2, {42}}}},
            {cell(2), {{9, {42}}}},
            {cell(4), {{6, {}}}}
        };
        const std::list<Cell*> path {cell(0), cell(1), cell(1), cell(2), cell(3), cell(4), cell(4)};
        const auto original = occupants;
        allocation_test::remaining = 0;
        const int count = checkNewConflictsQuantity(occupants, path, 10);
        allocation_test::remaining = -1;
        requireTest(count == 2, "Vertex conflict count must count each conflicting time only once.");
        requireTest(checkNewConflictsQuantity(occupants, {cell(0)}, 10) == 0,
            "The agent's own reservation counted as a conflict.");
        requireTest(checkNewConflictsQuantity(occupants, {cell(0)}, 42) == 1,
            "A different agent's reservation was ignored.");
        requireTest(checkNewConflictsQuantity(occupants, {}, 10) == 0 &&
            checkNewConflictsQuantity({}, path, 10) == 0, "Empty input reported conflicts.");
        requireTest(occupants == original, "Conflict counting changed reservations.");
    }

    // Transactions preserve unrelated occupants in a shared cell/time without
    // copying their containers, and restore nested nodes after group replacement.
    {
        Grid grid(3, 4);
        auto cell = [&](int x, int y) { return grid.getCellPtr(x, y); };
        auto* a = cell(1, 1);
        auto* b = cell(2, 1);
        const Paths paths {{cell(0, 1), a, b, cell(3, 1)},
            {cell(1, 0), a, b, cell(2, 2)}};
        const Paths replacements {{cell(0, 1), cell(0, 1), a, b, cell(3, 1)},
            {cell(1, 0), cell(1, 0), a, b, cell(2, 2)}};
        const std::vector<Agent> agents {
            agent(10, paths[0].front(), paths[0].back()),
            agent(42, paths[1].front(), paths[1].back())};
        const auto original = buildReservationState(grid, agents, paths);
        auto state = original;
        const auto* shared = &state.vertex_agents.at(a).at(1);
        const int* outsider = &*shared->find(42);
        {
            ReservationTransaction transaction(state);
            transaction.removePath(paths[0], 10);
            requireTest(&state.vertex_agents.at(a).at(1) == shared &&
                &*state.vertex_agents.at(a).at(1).find(42) == outsider,
                "Transaction copied an unrelated occupant or its time entry.");
            requireSameState(state, buildReservationState(grid, agents, paths, 0));
            transaction.insertPath(replacements[0], 10);
            requireTest(&state.vertex_agents.at(a).at(1) == shared,
                "Insertion copied other times of the same cell.");
        }
        requireSameState(state, original);

        // Fail each allocation in turn. Rollback runs while allocations remain
        // disabled, so a hidden allocation during unwinding also fails the test.
        bool completed = false;
        for (int failure = 0; failure < 2000; ++failure) {
            state = original;
            allocation_test::remaining = failure;
            try {
                ReservationTransaction transaction(state);
                transaction.removePath(paths[0], 10);
                transaction.removePath(paths[1], 42);
                transaction.insertPath(replacements[0], 10);
                transaction.insertPath(replacements[1], 42);
                completed = true;
            } catch (const std::bad_alloc&) {
            }
            allocation_test::remaining = -1;
            requireSameState(state, original);
            if (completed) break;
        }
        requireTest(completed, "Allocation-failure sweep never completed the transaction.");
        {
            ReservationTransaction transaction(state);
            transaction.removePath(paths[0], 10);
            transaction.removePath(paths[1], 42);
            transaction.insertPath(replacements[0], 10);
            transaction.insertPath(replacements[1], 42);
            transaction.commit();
        }
        requireSameState(state, buildReservationState(grid, agents, replacements));
        requireOccupancyMatchesPaths(grid, state, replacements);
    }

    // Shared vertices/movements survive one removal; reverse direction is independent.
    {
        Grid grid(3, 4);
        auto* left = grid.getCellPtr(0, 1);
        auto* a = grid.getCellPtr(1, 1);
        auto* b = grid.getCellPtr(2, 1);
        auto* right = grid.getCellPtr(3, 1);
        auto* top = grid.getCellPtr(1, 0);
        auto* bottom = grid.getCellPtr(2, 2);
        Paths paths {{left, a, b, right}, {top, a, b, bottom}, {bottom, b, a, left}};
        std::vector<Agent> agents {agent(10, left, right), agent(42, top, bottom), agent(-7, bottom, left)};
        const auto committed = buildReservationState(grid, agents, paths);
        auto state = committed;
        auto* untouched = grid.getCellPtr(0, 2);
        requireTest(!state.safeIntervalTable.safeIntervalsByCell.contains(untouched), "Unused cell was materialized.");
        repairSafeIntervalTable(state, paths[0], 10);
        requireSameState(state, buildReservationState(grid, agents, paths, 0));
        requireTest(state.vertex_agents.at(a).at(1) == std::unordered_set<int>{42}, "Removed the remaining occupant.");
        requireTest(state.safeIntervalTable.blockedEdgeArrivals.at({b, a}).contains(2), "Shared movement was freed.");
        repairSafeIntervalTable(state, paths[1], 42);
        auto excluded = paths;
        excluded[0].clear();
        excluded[1].clear();
        requireSameState(state, buildReservationState(grid, agents, excluded));
        requireTest(!state.safeIntervalTable.blockedEdgeArrivals.contains({b, a}), "Last edge contribution remains.");
        requireTest(state.safeIntervalTable.blockedEdgeArrivals.at({a, b}).contains(2), "Opposite direction was erased.");
        requireIntervals(state, b, {{0, 0}, {2, infinity}});
        updateReservationState(state, paths[0], 10);
        updateReservationState(state, paths[1], 42);
        requireSameState(state, committed);
        requireTest(!state.safeIntervalTable.safeIntervalsByCell.contains(untouched), "Unused cell was materialized by an update.");
        requireOccupancyMatchesPaths(grid, state, paths);

        // A full-repair group excludes both old paths, then inserts replacements
        // with shifted visits/goals while preserving the outsider's opposite move.
        for (std::size_t i : {0U, 1U}) {
            repairSafeIntervalTable(state, paths[i], agents[i].id);
        }
        requireSameState(state, buildReservationState(grid, agents, excluded));
        requireOccupancyMatchesPaths(grid, state, excluded);
        const Paths replacements {{left, left, a, b, right}, {top, top, top, a, b, bottom}};
        for (std::size_t i : {0U, 1U}) {
            updateReservationState(state, replacements[i], agents[i].id);
            excluded[i] = replacements[i];
            requireSameState(state, buildReservationState(grid, agents, excluded));
            requireOccupancyMatchesPaths(grid, state, excluded);
        }
        requireTest(state.vertex_agents.at(a).at(2) == std::unordered_set<int>({10, -7}),
            "Replacement removed the outsider's shared visit.");
        requireTest(!state.vertex_agents.at(a).contains(1), "Obsolete group occupancy survived.");
        requireTest(state.safeIntervalTable.blockedEdgeArrivals.at({a, b}).contains(2),
            "Group insertion lost the outsider's opposite movement.");
        requireTest(state.goal_reservations.at(right).arrivalTime == 4 &&
            state.goal_reservations.at(bottom).arrivalTime == 5, "Group arrival times were not shifted.");
        requireSameState(committed, buildReservationState(grid, agents, paths));
    }

    // Removing a permanent owner reveals a later visitor; removing the visitor
    // instead never clears the owner's implicit permanent occupancy.
    {
        Grid grid(2, 3);
        auto* start = grid.getCellPtr(0, 0);
        auto* a = grid.getCellPtr(1, 0);
        auto* b = grid.getCellPtr(2, 0);
        auto* y = grid.getCellPtr(2, 1);
        Paths paths {{start, start, start, start, a}, {y, y, y, y, y, y, b, a, b}};
        std::vector<Agent> agents {agent(10, start, a), agent(42, y, b)};
        const auto committed = buildReservationState(grid, agents, paths);
        requireIntervals(committed, a, {{0, 3}});
        auto state = committed;
        repairSafeIntervalTable(state, paths[0], 10);
        requireIntervals(state, a, {{0, 6}, {8, infinity}});
        requireSameState(state, buildReservationState(grid, agents, paths, 0));
        paths[0] = {start, start, a};
        updateReservationState(state, paths[0], 10);
        requireIntervals(state, a, {{0, 1}});
        requireSameState(state, buildReservationState(grid, agents, paths));
        repairSafeIntervalTable(state, paths[1], 42);
        requireIntervals(state, a, {{0, 1}});
        requireTest(!state.vertex_agents.at(a).contains(7), "Removed visitor remains explicit.");
        requireTest(state.goal_reservations.at(a).agentId == 10, "Visitor removal lost the parked owner.");
    }

    // An interior deletion creates a singleton; further deletions join intervals.
    {
        Grid grid(3, 3);
        auto* left = grid.getCellPtr(0, 1);
        auto* a = grid.getCellPtr(1, 1);
        auto* right = grid.getCellPtr(2, 1);
        auto* top = grid.getCellPtr(1, 0);
        auto* bottom = grid.getCellPtr(1, 2);
        Paths paths {{left, left, left, left, left, a, left, a, right},
            {top, top, top, top, top, top, a, bottom}};
        std::vector<Agent> agents {agent(10, left, right), agent(42, top, bottom)};
        auto state = buildReservationState(grid, agents, paths);
        requireIntervals(state, a, {{0, 4}, {8, infinity}});
        repairSafeIntervalTable(state, paths[1], 42);
        requireIntervals(state, a, {{0, 4}, {6, 6}, {8, infinity}});
        updateReservationState(state, paths[1], 42);
        repairSafeIntervalTable(state, paths[0], 10);
        requireIntervals(state, a, {{0, 5}, {7, infinity}});
        repairSafeIntervalTable(state, paths[1], 42);
        requireIntervals(state, a, {{0, infinity}});
        requireSameState(state, buildReservationState(grid, {}, {}));
    }

    // Contract failures are discovered before mutation. Empty paths are no-ops.
    {
        Grid grid(1, 3);
        auto* a = grid.getCellPtr(0, 0);
        auto* b = grid.getCellPtr(1, 0);
        auto* c = grid.getCellPtr(2, 0);
        Paths paths {{a, b, c}, {}};
        std::vector<Agent> agents {agent(42, a, c), agent(10, b, a)};
        const auto committed = buildReservationState(grid, agents, paths);
        auto state = committed;
        requireThrows<std::logic_error>([&] { repairSafeIntervalTable(state, paths[0], 0); });
        requireThrows<std::logic_error>([&] { repairSafeIntervalTable(state, {a, a, c}, 42); });
        requireThrows<std::logic_error>([&] { repairSafeIntervalTable(state, {b, c}, 42); });
        requireThrows<std::logic_error>([&] { updateReservationState(state, {b, c}, 10); });
        requireThrows<std::logic_error>([&] { updateReservationState(state, {a}, 42); });
        requireThrows<std::invalid_argument>([&] { repairSafeIntervalTable(state, {nullptr}, 42); });
        requireThrows<std::invalid_argument>([&] { updateReservationState(state, {a, nullptr}, 10); });
        repairSafeIntervalTable(state, {}, 42);
        updateReservationState(state, {}, 42);
        requireSameState(state, committed);
        requireSameState(state, buildReservationState(grid, agents, paths, 1));
        repairSafeIntervalTable(state, paths[0], 42);
        const auto empty = state;
        requireThrows<std::logic_error>([&] { repairSafeIntervalTable(state, paths[0], 42); });
        requireSameState(state, empty);
        updateReservationState(state, {a}, 42);
        requireIntervals(state, a, {});
        requireTest(state.goal_reservations.at(a).arrivalTime == 0 &&
            state.vertex_agents.at(a).at(0).contains(42), "A unit path lost its explicit or permanent ownership.");
        requireTest(state.safeIntervalTable.blockedEdgeArrivals.empty(), "A unit path registered an edge.");
    }

    // A synthetic far-future owner exercises sentinel arithmetic without a huge list.
    {
        Grid grid(1, 3);
        auto* a = grid.getCellPtr(1, 0);
        std::list<Cell*> path {grid.getCellPtr(0, 0), a, grid.getCellPtr(2, 0)};
        auto state = buildReservationState(grid, {}, {});
        state.vertex_agents[a][infinity - 1].insert(10);
        state.goal_reservations[a] = {10, infinity - 1};
        updateReservationState(state, path, 42);
        requireIntervals(state, a, {{0, 0}, {2, infinity - 2}});
        repairSafeIntervalTable(state, path, 42);
        requireIntervals(state, a, {{0, infinity - 2}});
    }

    // Repeated, reproducible replacement sequences catch stale shifted tails,
    // cross-agent multiplicity, waits, revisits and empty slots.
    {
        Grid grid(6, 6);
        std::mt19937 random(8042);
        const std::vector<int> ids {10, 42, -7, 91, 305};
        std::vector<Agent> agents;
        Paths paths;
        for (std::size_t i = 0; i < ids.size(); ++i) {
            auto* start = grid.getCellPtr(static_cast<int>(i), 0);
            auto* goal = grid.getCellPtr(5 - static_cast<int>(i), 5);
            agents.push_back(agent(ids[i], start, goal));
            paths.push_back(randomPath(grid, start, goal, random));
        }
        auto state = buildReservationState(grid, agents, paths);
        for (int iteration = 0; iteration < 100; ++iteration) {
            const auto i = static_cast<std::size_t>(iteration) % ids.size();
            repairSafeIntervalTable(state, paths[i], ids[i]);
            requireSameState(state, buildReservationState(grid, agents, paths, i));
            const auto& a = agents[i];
            paths[i] = iteration % 13 == 0 ? std::list<Cell*>{} : randomPath(grid,
                grid.getCellPtr(a.startPosition.x, a.startPosition.y),
                grid.getCellPtr(a.goalPosition.x, a.goalPosition.y), random);
            updateReservationState(state, paths[i], ids[i]);
            requireSameState(state, buildReservationState(grid, agents, paths));
            requireOccupancyMatchesPaths(grid, state, paths);
        }
    }

    // An impossible component precedes a repairable crossing. Both adapters must
    // preserve reservations on failure.
    for (auto strategy : {LocalRepairStrategy::RESOLVE_BY_AGENT, LocalRepairStrategy::RESOLVE_BY_TIME}) {
        std::vector<std::vector<int>> freeCells {{0, 0, 0, 1, 1, 1}, {1, 1, 0, 1, 1, 1}, {0, 0, 0, 1, 1, 1}};
        Grid coordinates(3, 6);
        std::vector<Agent> agents {
            agent(10, coordinates.getCellPtr(0, 1), coordinates.getCellPtr(1, 1)),
            agent(42, coordinates.getCellPtr(1, 1), coordinates.getCellPtr(0, 1)),
            agent(91, coordinates.getCellPtr(3, 1), coordinates.getCellPtr(5, 1)),
            agent(305, coordinates.getCellPtr(4, 0), coordinates.getCellPtr(4, 2))
        };
        Instance instance(&freeCells, 3, 6, agents);
        Grid& grid = const_cast<Grid&>(instance.getGrid());
        const auto iterative = LocalPathRepairIterativeSolver(instance, strategy).solve();
        const auto parallel = LocalPathRepairParallelSolver(instance, 3, strategy).solve();
        requireTest(!iterative.metrics.success && !parallel.metrics.success, "An impossible component succeeded.");
        requireTest(iterative.paths == parallel.paths, "Adapters differ after rejection.");
        requireSameState(iterative.reservations, parallel.reservations);
        requireSameState(iterative.reservations, buildReservationState(grid, instance.getAgents(), iterative.paths));
        if (strategy == LocalRepairStrategy::RESOLVE_BY_AGENT) {
            requireTest(iterative.paths == iterative.initialPaths, "Rejection partially committed a path.");
        } else {
            requireTest(iterative.remainingConflicts.vertexEvents.empty(), "The crossing was not repaired.");
        }
    }

    // The nearest anchor delays the suffix into a second crossing. An earlier
    // anchor avoids both crossings and preserves the initial wait; a full
    // replan would remove that wait, so the expected path distinguishes them.
    for (auto strategy : {LocalRepairStrategy::RESOLVE_BY_AGENT, LocalRepairStrategy::RESOLVE_BY_TIME}) {
        std::vector<std::vector<int>> freeCells(3, std::vector<int>(6, 1));
        Grid coordinates(3, 6);
        const std::vector<Agent> agents {
            agent(10, coordinates.getCellPtr(0, 1), coordinates.getCellPtr(5, 1)),
            agent(42, coordinates.getCellPtr(2, 0), coordinates.getCellPtr(2, 2)),
            agent(91, coordinates.getCellPtr(4, 0), coordinates.getCellPtr(4, 2))
        };
        Instance instance(&freeCells, 3, 6, agents);
        Grid& grid = instance.getGrid();
        auto cell = [&](int x, int y) { return grid.getCellPtr(x, y); };
        const Paths initialPaths {
            {cell(0, 1), cell(0, 1), cell(1, 1), cell(1, 1), cell(2, 1), cell(3, 1), cell(4, 1), cell(5, 1)},
            {cell(2, 0), cell(2, 0), cell(2, 0), cell(2, 0), cell(2, 1), cell(2, 2)},
            {cell(4, 0), cell(4, 0), cell(4, 0), cell(4, 0), cell(4, 0), cell(4, 0), cell(4, 0), cell(4, 1), cell(4, 2)}
        };
        const auto others = buildReservationState(grid, instance.getAgents(), initialPaths, 0);
        AStarSippSolver sipp;
        std::list<Cell*> nearest {cell(0, 1), cell(0, 1), cell(1, 1)};
        const auto bridge = sipp.solve(grid, cell(1, 1), cell(3, 1),
            others.safeIntervalTable, 3, AStarSippSolver::GoalOccupation::Transient);
        requireTest(!bridge.empty(), "Nearest-anchor fixture has no bridge.");
        nearest.insert(nearest.end(), bridge.begin(), bridge.end());
        nearest.insert(nearest.end(), {cell(4, 1), cell(5, 1)});
        requireTest(checkNewConflictsQuantity(others.vertex_agents, nearest, instance.getAgents()[0].id) == 1,
            "Nearest-anchor fixture must shift the suffix into a conflict.");

        Paths expected = initialPaths;
        expected[0] = {cell(0, 1), cell(0, 1), cell(1, 1), cell(2, 1), cell(3, 1), cell(4, 1), cell(5, 1)};
        requireTest(validateSolution(expected), "Earlier-anchor fixture is not solvable.");
        const auto result = repairInitialPaths(instance, initialPaths,
            std::chrono::steady_clock::now(), strategy);
        requireTest(result.metrics.success && result.paths == expected,
            "Repair did not reject the nearest anchor and accept the earlier safe anchor.");
        requireTest(result.initialPaths == initialPaths, "Anchor retries changed the initial paths.");
        requireSameState(result.reservations, buildReservationState(grid, instance.getAgents(), result.paths));
    }

    // Both local anchors reconnect before an unsafe suffix. Exhausting them
    // must discard that suffix and use the complete start-to-goal SIPP path.
    for (auto strategy : {LocalRepairStrategy::RESOLVE_BY_AGENT, LocalRepairStrategy::RESOLVE_BY_TIME}) {
        std::vector<std::vector<int>> freeCells(3, std::vector<int>(6, 1));
        Grid coordinates(3, 6);
        const std::vector<Agent> agents {
            agent(10, coordinates.getCellPtr(0, 1), coordinates.getCellPtr(5, 1)),
            agent(42, coordinates.getCellPtr(2, 0), coordinates.getCellPtr(2, 2)),
            agent(91, coordinates.getCellPtr(4, 2), coordinates.getCellPtr(3, 0))
        };
        Instance instance(&freeCells, 3, 6, agents);
        Grid& grid = instance.getGrid();
        auto cell = [&](int x, int y) { return grid.getCellPtr(x, y); };
        const Paths initialPaths {
            {cell(0, 1), cell(1, 1), cell(2, 1), cell(3, 1), cell(4, 1), cell(4, 0), cell(5, 0), cell(5, 1)},
            {cell(2, 0), cell(2, 0), cell(2, 1), cell(2, 2)},
            {cell(4, 2), cell(4, 2), cell(4, 2), cell(4, 2), cell(4, 2), cell(4, 1), cell(3, 1), cell(3, 0)}
        };
        const auto others = buildReservationState(grid, instance.getAgents(), initialPaths, 0);
        AStarSippSolver sipp;
        for (int anchor : {1, 0}) {
            std::list<Cell*> candidate;
            if (anchor == 1) candidate.push_back(cell(0, 1));
            const auto bridge = sipp.solve(grid, cell(anchor, 1), cell(3, 1),
                others.safeIntervalTable, anchor, AStarSippSolver::GoalOccupation::Transient);
            requireTest(!bridge.empty(), "Fallback fixture has no local bridge.");
            candidate.insert(candidate.end(), bridge.begin(), bridge.end());
            candidate.insert(candidate.end(), {cell(4, 1), cell(4, 0), cell(5, 0), cell(5, 1)});
            requireTest(checkNewConflictsQuantity(others.vertex_agents, candidate, instance.getAgents()[0].id) >= 1,
                "Every fallback fixture anchor must produce a vertex conflict.");
        }
        Paths expected = initialPaths;
        expected[0] = sipp.solve(grid, cell(0, 1), cell(5, 1), others.safeIntervalTable,
            0, AStarSippSolver::GoalOccupation::Permanent);
        requireTest(validateSolution(expected), "Complete-replan fixture is not solvable.");
        const auto result = repairInitialPaths(instance, initialPaths,
            std::chrono::steady_clock::now(), strategy);
        requireTest(result.metrics.success && result.paths == expected,
            "Repair did not use the full replan after rejecting every local anchor.");
        requireTest(result.initialPaths == initialPaths, "Full replan changed the initial paths.");
        requireSameState(result.reservations, buildReservationState(grid, instance.getAgents(), result.paths));
    }

    // A collision at the first agent's goal requires a complete fallback.
    for (auto strategy : {LocalRepairStrategy::RESOLVE_BY_AGENT, LocalRepairStrategy::RESOLVE_BY_TIME}) {
        std::vector<std::vector<int>> freeCells(2, std::vector<int>(2, 1));
        Grid coordinates(2, 2);
        std::vector<Agent> agents {
            agent(10, coordinates.getCellPtr(0, 0), coordinates.getCellPtr(1, 0)),
            agent(42, coordinates.getCellPtr(1, 1), coordinates.getCellPtr(0, 0))
        };
        Instance instance(&freeCells, 2, 2, agents);
        Grid& grid = instance.getGrid();
        const Paths initialPaths {
            {grid.getCellPtr(0, 0), grid.getCellPtr(1, 0)},
            {grid.getCellPtr(1, 1), grid.getCellPtr(1, 0), grid.getCellPtr(0, 0)}
        };
        Paths fullDetour = initialPaths;
        fullDetour[0] = {grid.getCellPtr(0, 0), grid.getCellPtr(0, 1),
            grid.getCellPtr(1, 1), grid.getCellPtr(1, 0)};
        requireTest(validateSolution(fullDetour), "The full-detour fixture is not solvable.");

        const auto result = repairInitialPaths(instance, initialPaths,
            std::chrono::steady_clock::now(), strategy);
        requireTest(result.metrics.success && validateSolution(result.paths),
            "Complete fallback did not repair the goal conflict.");
        requireSameState(result.reservations, buildReservationState(grid, instance.getAgents(), result.paths));
    }
    return 0;
}
