#include "mapf/core/grid.hpp"
#include "mapf/utils.hpp"

#include "test_support.hpp"

#include <list>
#include <unordered_set>
#include <variant>
#include <vector>

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

    return 0;
}
