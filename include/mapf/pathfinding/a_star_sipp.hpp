#pragma once

#include <list>
#include <vector>

#include "mapf/core/cell.hpp"
#include "mapf/core/grid.hpp"
#include "mapf/pathfinding/sipp/safe_interval_table.hpp"

namespace mapf {

    class AStarSippSolver {
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
    };

}
