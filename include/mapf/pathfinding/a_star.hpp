#pragma once

#include <list>

#include "mapf/core/cell.hpp"
#include "mapf/core/grid.hpp"

namespace mapf {

    class AStarSolver {
        public:
            std::list<Cell*> solve(Grid& grid, Cell* start, Cell* goal);
    };

}
