#include <iostream>

#include "mapf/core/grid.hpp"
#include "mapf/core/cell.hpp"

#include <list>

int main() {
    
    // teste A* comum
    
    int rows = 6;
    int cols = 6;

    std::vector<std::vector<int>> free_cells = {
        {1, 1, 1, 1, 1, 1},
        {1, 0, 1, 1, 1, 1},
        {1, 0, 0, 0, 1, 1},
        {1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1},
    };

    mapf::Grid grid(&free_cells, rows, cols);

    mapf::Cell* start = grid.getCellPtr(4, 3);
    mapf::Cell* goal = grid.getCellPtr(1, 2);

    std::list<mapf::Cell*> path = grid.getAStarPath(start, goal);

    for (mapf::Cell* cell : path) {
        mapf::printCell(cell);
    }

    return 0;
}

