#include "mapf/pathfinding/a_star_sipp.hpp"

#include "test_support.hpp"

#include <list>
#include <vector>

namespace {
    mapf::SafeIntervalTable fullySafeTable(mapf::Grid& grid) {
        mapf::SafeIntervalTable table;
        for (mapf::Cell& cell : grid.getCells()) {
            table.safeIntervalsByCell[&cell] = {{0, mapf::SAFE_INTERVAL_INFINITY}};
        }
        return table;
    }
}

int main() {
    // Scenario: a SIPP segment starts at absolute time 5. Expected: it arrives after two moves at absolute time 7.
    {
        mapf::Grid grid(1, 3);
        mapf::SafeIntervalTable table = fullySafeTable(grid);
        mapf::AStarSippSolver solver;
        std::list<mapf::Cell*> path = solver.solve(
            grid,
            grid.getCellPtr(0, 0),
            grid.getCellPtr(2, 0),
            table,
            5
        );

        requireTest(path.size() == 3, "Nonzero start time changed the geometric shortest path.");
        requireTest(path.front() == grid.getCellPtr(0, 0), "The segment must include its start cell.");
        requireTest(path.back() == grid.getCellPtr(2, 0), "The segment must reach its goal cell.");
    }

    // Scenario: the start cell is not safe at the requested absolute time. Expected: SIPP returns an empty segment.
    {
        mapf::Grid grid(1, 2);
        mapf::SafeIntervalTable table = fullySafeTable(grid);
        table.safeIntervalsByCell[grid.getCellPtr(0, 0)] = {{0, 4}};
        mapf::AStarSippSolver solver;

        requireTest(
            solver.solve(
                grid,
                grid.getCellPtr(0, 0),
                grid.getCellPtr(1, 0),
                table,
                5
            ).empty(),
            "SIPP accepted a blocked start time."
        );
    }

    // Scenario: the middle cell is blocked at the immediate arrival time. Expected: reconstruction includes one explicit wait.
    {
        mapf::Grid grid(1, 3);
        mapf::SafeIntervalTable table = fullySafeTable(grid);
        table.safeIntervalsByCell[grid.getCellPtr(1, 0)] = {
            {0, 5},
            {7, mapf::SAFE_INTERVAL_INFINITY}
        };
        mapf::AStarSippSolver solver;
        std::list<mapf::Cell*> path = solver.solve(
            grid,
            grid.getCellPtr(0, 0),
            grid.getCellPtr(2, 0),
            table,
            5
        );
        std::vector<mapf::Cell*> indexed(path.begin(), path.end());

        requireTest(indexed.size() == 4, "SIPP did not reconstruct the required wait.");
        requireTest(indexed[0] == indexed[1], "The required wait was not represented explicitly.");
    }

    // Scenario: the direct edge arrival is blocked at time 1. Expected: SIPP waits and traverses the edge at time 2.
    {
        mapf::Grid grid(1, 2);
        mapf::SafeIntervalTable table = fullySafeTable(grid);
        mapf::Cell* start = grid.getCellPtr(0, 0);
        mapf::Cell* goal = grid.getCellPtr(1, 0);
        table.blockedEdgeArrivals[{start, goal}].insert(1);
        mapf::AStarSippSolver solver;
        std::list<mapf::Cell*> path = solver.solve(grid, start, goal, table, 0);
        std::vector<mapf::Cell*> indexed(path.begin(), path.end());

        requireTest(indexed.size() == 3, "SIPP did not avoid the blocked reverse-edge arrival.");
        requireTest(indexed[0] == start && indexed[1] == start && indexed[2] == goal, "Unexpected edge-wait path.");
    }

    // Scenario: a transient suffix endpoint is free early but blocked later. Expected: the table overload accepts the earliest arrival.
    {
        mapf::Grid grid(2, 2);
        mapf::Cell* start = grid.getCellPtr(0, 0);
        mapf::Cell* goal = grid.getCellPtr(1, 0);
        mapf::Cell* auxiliary = grid.getCellPtr(1, 1);
        std::vector<std::list<mapf::Cell*>> reservations {
            {auxiliary, auxiliary, goal, auxiliary}
        };
        mapf::AStarSippSolver solver;
        mapf::SafeIntervalTable table = solver.getSafeIntervalsByCell(grid, reservations);
        std::list<mapf::Cell*> transientPath = solver.solve(grid, start, goal, table, 0);

        requireTest(transientPath.size() == 2, "A transient endpoint did not use its earliest safe interval.");
    }

    // Scenario: a complete path reaches its real goal before a later reservation. Expected: permanent-goal planning delays arrival until the infinite interval.
    {
        mapf::Grid grid(2, 2);
        mapf::Cell* start = grid.getCellPtr(0, 0);
        mapf::Cell* goal = grid.getCellPtr(1, 0);
        mapf::Cell* auxiliary = grid.getCellPtr(1, 1);
        std::vector<std::list<mapf::Cell*>> reservations {
            {auxiliary, auxiliary, goal, auxiliary}
        };
        mapf::AStarSippSolver solver;
        std::list<mapf::Cell*> permanentPath = solver.solve(
            grid,
            start,
            goal,
            reservations
        );

        requireTest(permanentPath.size() == 4, "Permanent-goal SIPP accepted a finite goal interval.");
        requireTest(permanentPath.back() == goal, "Permanent-goal SIPP did not reach the goal.");
    }

    // Scenario: start and transient goal are the same safe cell. Expected: SIPP returns a one-cell segment.
    {
        mapf::Grid grid(1, 1);
        mapf::SafeIntervalTable table = fullySafeTable(grid);
        mapf::AStarSippSolver solver;
        std::list<mapf::Cell*> path = solver.solve(
            grid,
            grid.getCellPtr(0, 0),
            grid.getCellPtr(0, 0),
            table,
            9
        );

        requireTest(path.size() == 1, "Start-equals-goal did not return a one-cell segment.");
    }

    // Scenario: the requested start time is negative. Expected: SIPP rejects the request with an empty segment.
    {
        mapf::Grid grid(1, 1);
        mapf::SafeIntervalTable table = fullySafeTable(grid);
        mapf::AStarSippSolver solver;

        requireTest(
            solver.solve(
                grid,
                grid.getCellPtr(0, 0),
                grid.getCellPtr(0, 0),
                table,
                -1
            ).empty(),
            "SIPP accepted a negative absolute start time."
        );
    }

    return 0;
}
