#pragma once

#include <list>
#include <vector>

#include "mapf/core/cell.hpp"
#include "mapf/core/grid.hpp"
#include "mapf/pathfinding/sipp/safe_interval_table.hpp"

namespace mapf {

    class AStarSippSolver {
        public:
            enum class GoalOccupation {
                Transient,
                Permanent
            };

        private:
            std::list<Cell*> solveWithTable(
                Grid& grid,
                Cell* start,
                Cell* goal,
                const SafeIntervalTable& safeIntervalTable,
                int startTime,
                GoalOccupation goalOccupation
            );

        public:
            SafeIntervalTable getSafeIntervalsByCell(
                Grid& grid,
                const std::vector<std::list<Cell*>>& otherAgentPaths
            );

            std::list<Cell*> solve(
                Grid& grid,
                Cell* start,
                Cell* goal,
                const std::vector<std::list<Cell*>>& otherAgentPaths
            );

            std::list<Cell*> solve(
                Grid& grid,
                Cell* start,
                Cell* goal,
                const SafeIntervalTable& safeIntervalTable,
                int startTime
            );

            std::list<Cell*> solve(
                Grid& grid,
                Cell* start,
                Cell* goal,
                const SafeIntervalTable& safeIntervalTable,
                int startTime,
                GoalOccupation goalOccupation
            );
    };

}
