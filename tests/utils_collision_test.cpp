#include "mapf/core/grid.hpp"
#include "mapf/utils.hpp"

#include "test_support.hpp"

#include <algorithm>
#include <list>
#include <vector>

namespace {
    bool hasCellConflictAt(const mapf::SolutionConflicts& conflicts, int time) {
        return std::any_of(
            conflicts.cellConflicts.begin(),
            conflicts.cellConflicts.end(),
            [time](const mapf::CellConflict& conflict) {
                return conflict.time == time;
            }
        );
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
            grid.getCellPtr(1, 0)
        };

        mapf::SolutionConflicts forward = getCollision({shortPath, longPath});
        mapf::SolutionConflicts reverse = getCollision({longPath, shortPath});

        requireTest(hasCellConflictAt(forward, 3), "Forward stay-at-goal conflict was not reported.");
        requireTest(hasCellConflictAt(reverse, 3), "Reverse-order stay-at-goal conflict was not reported.");
    }

    // Scenario: two agents enter the same cell at the same timestep. Expected: one pairwise vertex conflict is reported.
    {
        mapf::Grid grid(1, 3);
        mapf::SolutionConflicts conflicts = getCollision({
            {grid.getCellPtr(0, 0), grid.getCellPtr(1, 0)},
            {grid.getCellPtr(2, 0), grid.getCellPtr(1, 0)}
        });

        requireTest(conflicts.cellConflicts.size() == 1, "Unexpected vertex-conflict count.");
        requireTest(conflicts.cellConflicts.front().time == 1, "Vertex conflict has the wrong time.");
    }

    // Scenario: two agents traverse one edge in opposite directions. Expected: one edge-swap conflict is reported.
    {
        mapf::Grid grid(1, 2);
        mapf::SolutionConflicts conflicts = getCollision({
            {grid.getCellPtr(0, 0), grid.getCellPtr(1, 0)},
            {grid.getCellPtr(1, 0), grid.getCellPtr(0, 0)}
        });

        requireTest(conflicts.edgeConflicts.size() == 1, "Unexpected edge-conflict count.");
        requireTest(conflicts.edgeConflicts.front().time == 1, "Edge conflict has the wrong time.");
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

        requireTest(&forward.edgeConflicts.front().cell_1 == &reverse.edgeConflicts.front().cell_1, "Edge endpoint order depends on path order.");
        requireTest(&forward.edgeConflicts.front().cell_2 == &reverse.edgeConflicts.front().cell_2, "Edge endpoint order depends on path order.");
    }

    // Scenario: three agents share one vertex at one timestep. Expected: all three unordered path pairs are reported.
    {
        mapf::Grid grid(2, 3);
        mapf::Cell* center = grid.getCellPtr(1, 0);
        mapf::SolutionConflicts conflicts = getCollision({
            {grid.getCellPtr(0, 0), center},
            {grid.getCellPtr(2, 0), center},
            {grid.getCellPtr(1, 1), center}
        });

        requireTest(conflicts.cellConflicts.size() == 3, "Three-agent pairwise conflicts were not all reported.");
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
        requireTest(conflicts.cellConflicts.empty(), "A conflict-free unequal-length solution has a vertex conflict.");
        requireTest(conflicts.edgeConflicts.empty(), "A conflict-free unequal-length solution has an edge conflict.");
        requireTest(validateSolution(paths), "validateSolution rejected a conflict-free solution.");
    }

    return 0;
}
