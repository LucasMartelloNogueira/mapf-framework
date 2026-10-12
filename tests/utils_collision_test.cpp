#include "mapf/core/grid.hpp"
#include "mapf/solvers/local_path_repair_solver.hpp"
#include "mapf/utils.hpp"

#include "test_support.hpp"

#include <algorithm>
#include <list>
#include <unordered_set>
#include <variant>
#include <vector>

namespace {
    void requireSameConflicts(const mapf::SolutionConflicts& actual, const mapf::SolutionConflicts& expected) {
        requireTest(actual.vertexEvents.size() == expected.vertexEvents.size(), "V2 vertex event counts differ.");
        for (const auto& [key, event] : expected.vertexEvents) {
            requireTest(actual.vertexEvents.at(key).participants == event.participants, "V2 vertex participants differ.");
        }
        requireTest(actual.edgeEvents.size() == expected.edgeEvents.size(), "V2 edge event counts differ.");
        for (const auto& [key, event] : expected.edgeEvents) {
            const auto& value = actual.edgeEvents.at(key);
            requireTest(value.forward == event.forward && value.reverse == event.reverse, "V2 edge participants differ.");
        }
        requireTest(actual.byAgent.size() == expected.byAgent.size(), "V2 agent slots differ.");
        for (std::size_t i = 0; i < expected.byAgent.size(); ++i) {
            requireTest(actual.byAgent[i].size() == expected.byAgent[i].size(), "V2 agent record counts differ.");
            for (std::size_t j = 0; j < expected.byAgent[i].size(); ++j) {
                requireTest(!mapf::conflictRecordComesBefore(actual.byAgent[i][j], expected.byAgent[i][j]) &&
                    !mapf::conflictRecordComesBefore(expected.byAgent[i][j], actual.byAgent[i][j]),
                    "V2 agent records differ or are out of order.");
            }
        }
    }

    mapf::PathReservationState explicitReservations(const std::vector<std::list<mapf::Cell*>>& paths) {
        mapf::PathReservationState state;
        for (std::size_t agent = 0; agent < paths.size(); ++agent) {
            int time = 0;
            for (mapf::Cell* cell : paths[agent]) {
                state.vertex_agents[cell][time++].insert(static_cast<int>(agent));
            }
        }
        return state;
    }
}

int main() {
    // Scenario: a longer path enters the permanent goal of a shorter path. Expected: the conflict is reported regardless of path order.
    {
        mapf::Grid grid(1, 4);
        std::list<mapf::Cell*> shortPath {
            grid.getCellPtr(0, 0),
            grid.getCellPtr(1, 0)
        };
        std::list<mapf::Cell*> longPath {
            grid.getCellPtr(3, 0),
            grid.getCellPtr(2, 0),
            grid.getCellPtr(2, 0),
            grid.getCellPtr(1, 0),
            grid.getCellPtr(0, 0)
        };

        mapf::SolutionConflicts forward = getCollision({shortPath, longPath});
        mapf::SolutionConflicts reverse = getCollision({longPath, shortPath});

        const mapf::CellTime key {grid.getCellPtr(1, 0), 3};
        requireTest(forward.vertexEvents.contains(key), "Forward stay-at-goal conflict was not reported.");
        requireTest(reverse.vertexEvents.contains(key), "Reverse-order stay-at-goal conflict was not reported.");
        requireTest(forward.vertexEvents.at(key).participants == std::unordered_set<int>({0, 1}), "Stay-at-goal conflict lost a participant.");
        requireTest(reverse.vertexEvents.at(key).participants == std::unordered_set<int>({0, 1}), "Reversed stay-at-goal conflict lost a participant.");
        requireTest(forward.byAgent.size() == 2 && reverse.byAgent.size() == 2, "Stay-at-goal repair records are not aligned with paths.");
        requireTest(forward.byAgent[0].empty() && reverse.byAgent[1].empty(), "A parked agent received an explicit repair record.");
        requireTest(forward.byAgent[1].size() == 1 && reverse.byAgent[0].size() == 1, "The moving agent lost its repair record.");
    }

    // Scenario: two agents enter the same cell at the same timestep and continue to distinct goals. Expected: one vertex event contains both participants.
    {
        mapf::Grid grid(1, 3);
        mapf::SolutionConflicts conflicts = getCollision({
            {grid.getCellPtr(0, 0), grid.getCellPtr(1, 0), grid.getCellPtr(2, 0)},
            {grid.getCellPtr(2, 0), grid.getCellPtr(1, 0), grid.getCellPtr(0, 0)}
        });

        const mapf::CellTime key {grid.getCellPtr(1, 0), 1};
        requireTest(conflicts.vertexEvents.size() == 1, "Unexpected vertex-conflict count.");
        requireTest(conflicts.vertexEvents.contains(key), "Vertex conflict has the wrong cell or time.");
        requireTest(conflicts.vertexEvents.at(key).participants == std::unordered_set<int>({0, 1}), "Vertex conflict has the wrong participants.");
    }

    // Scenario: two agents traverse one edge in opposite directions. Expected: one edge-swap conflict is reported.
    {
        mapf::Grid grid(1, 2);
        mapf::SolutionConflicts conflicts = getCollision({
            {grid.getCellPtr(0, 0), grid.getCellPtr(1, 0)},
            {grid.getCellPtr(1, 0), grid.getCellPtr(0, 0)}
        });

        const mapf::EdgeTime key {grid.getCellPtr(0, 0), grid.getCellPtr(1, 0), 1};
        requireTest(conflicts.edgeEvents.size() == 1, "Unexpected edge-conflict count.");
        requireTest(conflicts.edgeEvents.contains(key), "Edge conflict has the wrong endpoints or time.");
        requireTest(conflicts.edgeEvents.at(key).forward == std::unordered_set<int>({0}), "Edge conflict has the wrong forward participants.");
        requireTest(conflicts.edgeEvents.at(key).reverse == std::unordered_set<int>({1}), "Edge conflict has the wrong reverse participants.");
    }

    // Scenario: the two edge-swap paths are supplied in reverse order. Expected: the reported edge endpoints keep the same coordinate order.
    {
        mapf::Grid grid(1, 2);
        std::list<mapf::Cell*> leftToRight {
            grid.getCellPtr(0, 0),
            grid.getCellPtr(1, 0)
        };
        std::list<mapf::Cell*> rightToLeft {
            grid.getCellPtr(1, 0),
            grid.getCellPtr(0, 0)
        };
        mapf::SolutionConflicts forward = getCollision({leftToRight, rightToLeft});
        mapf::SolutionConflicts reverse = getCollision({rightToLeft, leftToRight});

        requireTest(forward.edgeEvents.size() == 1 && reverse.edgeEvents.size() == 1, "Reversing path order lost the edge conflict.");
        requireTest(forward.edgeEvents.begin()->first == reverse.edgeEvents.begin()->first, "Edge endpoint order depends on path order.");
        requireTest(reverse.edgeEvents.begin()->second.forward == std::unordered_set<int>({1}), "Reversed edge conflict has the wrong forward participants.");
        requireTest(reverse.edgeEvents.begin()->second.reverse == std::unordered_set<int>({0}), "Reversed edge conflict has the wrong reverse participants.");
    }

    // Scenario: three agents share one vertex at one timestep and continue to distinct goals. Expected: one event contains all participants and each agent has one repair record.
    {
        mapf::Grid grid(2, 3);
        mapf::Cell* center = grid.getCellPtr(1, 0);
        mapf::SolutionConflicts conflicts = getCollision({
            {grid.getCellPtr(0, 0), center, grid.getCellPtr(2, 0)},
            {grid.getCellPtr(2, 0), center, grid.getCellPtr(1, 1)},
            {grid.getCellPtr(1, 1), center, grid.getCellPtr(0, 0)}
        });

        const mapf::CellTime key {center, 1};
        requireTest(conflicts.vertexEvents.size() == 1, "Three-agent conflict was not grouped into one event.");
        requireTest(conflicts.vertexEvents.contains(key), "Three-agent conflict has the wrong cell or time.");
        requireTest(conflicts.vertexEvents.at(key).participants == std::unordered_set<int>({0, 1, 2}), "Three-agent conflict lost participants.");
        requireTest(conflicts.byAgent.size() == 3, "Repair records are not aligned with paths.");
        for (const std::vector<std::variant<mapf::CellConflict, mapf::EdgeConflict>>& records : conflicts.byAgent) {
            requireTest(records.size() == 1, "An agent did not receive exactly one repair record.");
            const mapf::CellConflict* record = std::get_if<mapf::CellConflict>(&records.front());
            requireTest(record != nullptr && record->cell == center && record->time == 1, "An agent received the wrong vertex repair record.");
        }
    }

    // Scenario: an empty path is mixed with one valid path. Expected: the empty path creates no occupancy or conflict.
    {
        mapf::Grid grid(1, 2);
        std::vector<std::list<mapf::Cell*>> paths {
            {},
            {grid.getCellPtr(0, 0), grid.getCellPtr(1, 0)}
        };

        requireTest(validateSolution(paths), "An empty path incorrectly created a collision.");
    }

    // Scenario: unequal path lengths end at different goals and never collide. Expected: validation succeeds through the makespan.
    {
        mapf::Grid grid(2, 3);
        std::vector<std::list<mapf::Cell*>> paths {
            {grid.getCellPtr(0, 0), grid.getCellPtr(1, 0)},
            {grid.getCellPtr(2, 1), grid.getCellPtr(1, 1), grid.getCellPtr(0, 1)}
        };

        mapf::SolutionConflicts conflicts = getCollision(paths);
        requireTest(conflicts.vertexEvents.empty(), "A conflict-free unequal-length solution has a vertex conflict.");
        requireTest(conflicts.edgeEvents.empty(), "A conflict-free unequal-length solution has an edge conflict.");
        requireTest(validateSolution(paths), "validateSolution rejected a conflict-free solution.");
    }

    // Scenario: either participant delays an edge swap. Expected: both obsolete repair records are removed while the remaining memberships and input are preserved.
    for (int index : {0, 1}) {
        mapf::Grid grid(1, 2);
        mapf::Cell* left = grid.getCellPtr(0, 0);
        mapf::Cell* right = grid.getCellPtr(1, 0);
        std::vector<std::list<mapf::Cell*>> paths {{left, right}, {right, left}};
        const mapf::SolutionConflicts original = getCollision(paths);
        mapf::Cell* start = paths[index].front();
        mapf::Cell* goal = paths[index].back();
        const mapf::SolutionConflicts updated = updateSolutionConflicts(
            paths[index], {start, start, goal}, index, original.byAgent[index].front(), original);

        const mapf::EdgeTime oldKey = mapf::makeEdgeTime(left, right, 1);
        const mapf::EdgeTime newKey = mapf::makeEdgeTime(left, right, 2);
        const auto& oldEdge = updated.edgeEvents.at(oldKey);
        const auto& newEdge = updated.edgeEvents.at(newKey);
        requireTest(!oldEdge.forward.contains(index) && !oldEdge.reverse.contains(index),
            "The old edge retained the repaired agent.");
        requireTest(oldEdge.forward.contains(1 - index) || oldEdge.reverse.contains(1 - index),
            "Updating an edge removed the other agent.");
        requireTest((index == 0 ? newEdge.forward : newEdge.reverse) == std::unordered_set<int>{index},
            "The new edge has the wrong direction or participants.");
        requireTest(!updated.edgeEvents.contains(mapf::makeEdgeTime(start, start, 1)),
            "Waiting created a movement edge.");
        requireTest(updated.vertexEvents.at({start, 1}).participants.contains(index) &&
            updated.vertexEvents.at({goal, 2}).participants.contains(index),
            "The delayed path lost its vertex memberships.");
        requireTest(updated.byAgent[index].empty() && updated.byAgent[1 - index].empty(),
            "A resolved edge swap retained an obsolete repair record.");
        requireTest(original.edgeEvents.at(oldKey).forward == std::unordered_set<int>{0} &&
            original.edgeEvents.at(oldKey).reverse == std::unordered_set<int>{1} &&
            original.vertexEvents.empty() && original.byAgent[index].size() == 1,
            "Updating conflicts mutated the original state.");
    }

    // Scenario: a detour removes one participant from a three-agent vertex event. Expected: the other participants and the order of unrelated records are preserved.
    {
        mapf::Grid grid(2, 3);
        mapf::Cell* left = grid.getCellPtr(0, 0);
        mapf::Cell* center = grid.getCellPtr(1, 0);
        mapf::Cell* right = grid.getCellPtr(2, 0);
        mapf::Cell* bottom = grid.getCellPtr(1, 1);
        const std::list<mapf::Cell*> oldPath {left, center, right};
        mapf::SolutionConflicts original = getCollision({oldPath, {right, center, bottom}, {bottom, center, left}});
        const mapf::EdgeConflict before {left, center, 0};
        const mapf::CellConflict after {right, 3};
        original.byAgent[0].insert(original.byAgent[0].begin(), before);
        original.byAgent[0].push_back(after);
        const mapf::SolutionConflicts updated = updateSolutionConflicts(
            oldPath, {left, grid.getCellPtr(0, 1), bottom, grid.getCellPtr(2, 1), right},
            0, mapf::CellConflict{center, 1}, original);

        requireTest(updated.vertexEvents.at({center, 1}).participants == std::unordered_set<int>({1, 2}),
            "A vertex update lost the other conflict participants.");
        requireTest(updated.vertexEvents.at({bottom, 2}).participants == std::unordered_set<int>{0},
            "The detour's vertex membership is missing.");
        requireTest(updated.byAgent[0].size() == 2 &&
            std::get<mapf::EdgeConflict>(updated.byAgent[0][0]).time == before.time &&
            std::get<mapf::CellConflict>(updated.byAgent[0][1]).time == after.time,
            "Filtering the selected conflict changed unrelated records or their order.");
        requireTest(updated.byAgent[1].size() == 1 && updated.byAgent[2].size() == 1,
            "Filtering one agent's conflicts changed another agent's records.");
        requireTest(original.vertexEvents.at({center, 1}).participants.contains(0) &&
            original.byAgent[0].size() == 3, "The vertex update mutated the original state.");
    }

    // Scenario: the selected conflict is missing, duplicated, or belongs to an invalid index. Expected: reject the call before sizing a smaller record vector.
    {
        mapf::Grid grid(1, 1);
        mapf::Cell* cell = grid.getCellPtr(0, 0);
        const mapf::CellConflict conflict {cell, 0};
        const auto requireRejected = [&](int index, const mapf::SolutionConflicts& conflicts) {
            bool rejected = false;
            try {
                updateSolutionConflicts({cell}, {cell}, index, conflict, conflicts);
            } catch (const std::invalid_argument&) {
                rejected = true;
            }
            requireTest(rejected, "Invalid conflict removal was accepted.");
        };
        mapf::SolutionConflicts conflicts;
        conflicts.byAgent.resize(1);
        requireRejected(0, conflicts);
        requireRejected(-1, conflicts);
        requireRejected(1, conflicts);
        conflicts.byAgent[0] = {mapf::CellConflict{cell, 1}};
        requireRejected(0, conflicts);
        conflicts.byAgent[0] = {conflict, conflict};
        requireRejected(0, conflicts);
    }

    // V2 creates a vertex conflict for all participants and removes it even
    // when the repaired path no longer visits its old cell.
    {
        mapf::Grid grid(2, 3);
        auto* left = grid.getCellPtr(0, 0);
        auto* center = grid.getCellPtr(1, 0);
        auto* right = grid.getCellPtr(2, 0);
        const std::list<mapf::Cell*> detour {
            left, grid.getCellPtr(0, 1), grid.getCellPtr(1, 1), grid.getCellPtr(2, 1), right
        };
        const std::vector<std::list<mapf::Cell*>> paths {{left, center, right}, {right, center, left}};
        const auto before = getCollision({detour, paths[1]});
        const auto state = explicitReservations(paths);
        const auto originalOccupancy = state.vertex_agents;
        const auto updated = UpdateSolutionConflictsV2(paths[0], state, before, 0, 0);
        requireSameConflicts(updated, getCollision(paths));
        requireTest(updated.byAgent[0].size() == 1 && updated.byAgent[1].size() == 1,
            "V2 did not add the vertex record to every participant.");
        const auto unchanged = UpdateSolutionConflictsV2(paths[0], state, updated, 0, 0);
        requireSameConflicts(unchanged, updated);
        const auto resolved = UpdateSolutionConflictsV2(detour, explicitReservations({detour, paths[1]}), updated, 0, 0);
        requireSameConflicts(resolved, before);
        requireTest(state.vertex_agents == originalOccupancy && before.empty(), "V2 mutated its inputs.");
    }

    // A new edge swap is registered for both directions. A detour removes both
    // obsolete records, including the edge no longer traversed by the repaired agent.
    for (int agentId : {0, 1}) {
        mapf::Grid grid(2, 2);
        auto* left = grid.getCellPtr(0, 0);
        auto* right = grid.getCellPtr(1, 0);
        std::vector<std::list<mapf::Cell*>> paths {{left, right}, {right, left}};
        mapf::SolutionConflicts before;
        before.byAgent.resize(2);
        const auto updated = UpdateSolutionConflictsV2(paths[agentId], explicitReservations(paths), before, agentId, 0);
        requireSameConflicts(updated, getCollision(paths));
        const auto repeated = UpdateSolutionConflictsV2(paths[agentId], explicitReservations(paths), updated, agentId, 0);
        requireSameConflicts(repeated, updated);
        const int startX = agentId == 0 ? 0 : 1;
        paths[agentId] = {grid.getCellPtr(startX, 0), grid.getCellPtr(startX, 1),
            grid.getCellPtr(1 - startX, 1), grid.getCellPtr(1 - startX, 0)};
        const auto resolved = UpdateSolutionConflictsV2(paths[agentId], explicitReservations(paths), updated, agentId, 0);
        requireSameConflicts(resolved, getCollision(paths));
        requireTest(resolved.empty(), "V2 retained a resolved edge swap.");
    }

    // Occupancy at opposite endpoints is insufficient for a swap: a legal cycle
    // has different agents arriving and departing each endpoint.
    {
        mapf::Grid grid(2, 2);
        auto* a = grid.getCellPtr(0, 0);
        auto* b = grid.getCellPtr(1, 0);
        auto* c = grid.getCellPtr(1, 1);
        auto* d = grid.getCellPtr(0, 1);
        const std::vector<std::list<mapf::Cell*>> paths {{a, b}, {b, c}, {c, d}, {d, a}};
        const auto before = getCollision(paths);
        const auto updated = UpdateSolutionConflictsV2(paths[0], explicitReservations(paths), before, 0, 0);
        requireSameConflicts(updated, before);
    }

    // Removing one forward participant leaves an active multi-agent swap.
    // Removing the final forward participant clears it for the reverse participant too.
    {
        mapf::Grid grid(3, 4);
        auto cell = [&](int x, int y) { return grid.getCellPtr(x, y); };
        auto* a = cell(1, 1);
        auto* b = cell(2, 1);
        std::vector<std::list<mapf::Cell*>> paths {
            {cell(0, 1), a, b, cell(3, 1)},
            {cell(1, 0), a, b, cell(2, 2)},
            {cell(2, 2), b, a, cell(0, 1)}
        };
        const auto before = getCollision(paths);
        paths[0] = {cell(0, 1), cell(0, 2), cell(1, 2), cell(2, 2), cell(3, 2), cell(3, 1)};
        const auto first = UpdateSolutionConflictsV2(paths[0], explicitReservations(paths), before, 0, 0);
        requireSameConflicts(first, getCollision(paths));
        const auto& edge = first.edgeEvents.at(mapf::makeEdgeTime(a, b, 2));
        requireTest(edge.forward == std::unordered_set<int>{1} && edge.reverse == std::unordered_set<int>{2},
            "V2 lost the remaining active edge participants.");
        paths[1] = {cell(1, 0), cell(2, 0), cell(3, 0), cell(3, 1), cell(3, 2), cell(2, 2)};
        const auto second = UpdateSolutionConflictsV2(paths[1], explicitReservations(paths), first, 1, 0);
        requireSameConflicts(second, getCollision(paths));
        requireTest(!second.edgeEvents.contains(mapf::makeEdgeTime(a, b, 2)),
            "V2 retained an edge with participants in only one direction.");
    }

    // Existing vertex conflicts refresh their participant sets even when the
    // analyzed agent remains involved. Unrelated records are preserved and sorted.
    {
        mapf::Grid grid(2, 3);
        auto* left = grid.getCellPtr(0, 0);
        auto* center = grid.getCellPtr(1, 0);
        auto* right = grid.getCellPtr(2, 0);
        const std::list<mapf::Cell*> path {left, center, right};
        mapf::SolutionConflicts before;
        before.byAgent.resize(4);
        before.vertexEvents[{center, 1}].participants = {0, 1, 2};
        for (int id : {0, 1, 2}) before.byAgent[id].push_back(mapf::CellConflict{center, 1});
        const mapf::CellConflict unrelated {grid.getCellPtr(1, 1), 3};
        before.vertexEvents[{unrelated.cell, unrelated.time}].participants = {1, 3};
        for (int id : {1, 3}) before.byAgent[id].push_back(unrelated);
        auto state = explicitReservations({path});
        state.vertex_agents[center][1] = {0, 2, 3};
        const auto updated = UpdateSolutionConflictsV2(path, state, before, 0, 0);
        requireTest(updated.vertexEvents.at({center, 1}).participants == std::unordered_set<int>({0, 2, 3}),
            "V2 did not refresh the participants of a continuing vertex conflict.");
        requireTest(updated.byAgent[1].size() == 1 && updated.byAgent[3].size() == 2 &&
            updated.vertexEvents.at({unrelated.cell, unrelated.time}).participants == std::unordered_set<int>({1, 3}),
            "V2 removed unrelated records or failed to synchronize participant changes.");
        requireTest(std::is_sorted(updated.byAgent[3].begin(), updated.byAgent[3].end(), mapf::conflictRecordComesBefore),
            "V2 inserted a record out of order.");
    }

    // Analyze a suffix without dropping earlier or unrelated conflicts. Empty
    // suffixes leave the entire input snapshot intact.
    {
        mapf::Grid grid(3, 4);
        auto cell = [&](int x, int y) { return grid.getCellPtr(x, y); };
        const std::vector<std::list<mapf::Cell*>> paths {
            {cell(0, 1), cell(1, 1), cell(2, 1), cell(3, 1)},
            {cell(1, 0), cell(1, 1), cell(2, 1), cell(2, 2)},
            {cell(2, 2), cell(2, 1), cell(1, 1), cell(0, 1)}
        };
        const auto state = explicitReservations(paths);
        const auto before = getCollision(paths);
        requireSameConflicts(UpdateSolutionConflictsV2(paths[0], state, before, 0, 2), before);
        requireSameConflicts(UpdateSolutionConflictsV2(paths[0], state, before, 0, 4), before);
    }

    // Empty/single-cell paths, waits, missing occupancy entries and invalid
    // indexes must not cause out-of-bounds reads or false conflicts.
    {
        mapf::Grid grid(1, 1);
        auto* cell = grid.getCellPtr(0, 0);
        mapf::SolutionConflicts before;
        before.byAgent.resize(1);
        for (const std::list<mapf::Cell*>& path : std::vector<std::list<mapf::Cell*>>{{}, {cell}, {cell, cell}}) {
            requireSameConflicts(UpdateSolutionConflictsV2(path, explicitReservations({path}), before, 0, 0), before);
        }
        mapf::SolutionConflicts stale;
        stale.byAgent.resize(2);
        stale.vertexEvents[{cell, 0}].participants = {0, 1};
        for (int id : {0, 1}) stale.byAgent[id].push_back(mapf::CellConflict{cell, 0});
        const auto cleared = UpdateSolutionConflictsV2({cell}, {}, stale, 0, 0);
        requireTest(cleared.empty() && cleared.byAgent[0].empty() && cleared.byAgent[1].empty(),
            "V2 failed to clear a conflict with no remaining occupancy.");
        for (const auto& [agentId, pathIndex] : std::vector<std::pair<int, int>>{{-1, 0}, {1, 0}, {0, -1}, {0, 2}}) {
            bool rejected = false;
            try {
                UpdateSolutionConflictsV2({cell}, explicitReservations({{cell}}), before, agentId, pathIndex);
            } catch (const std::invalid_argument&) {
                rejected = true;
            }
            requireTest(rejected, "V2 accepted an invalid agent or path index.");
        }
    }

    return 0;
}
