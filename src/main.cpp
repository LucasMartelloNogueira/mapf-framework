#include <iostream>
#include <list>
#include <vector>

#include "mapf/core/grid.hpp"
#include "mapf/core/cell.hpp"
#include "mapf/pathfinding/a_star.hpp"
#include "mapf/pathfinding/a_star_sipp.hpp"

int main() {
    
    // teste A* comum
    
    // int rows = 6;
    // int cols = 6;

    // std::vector<std::vector<int>> free_cells = {
    //     {1, 1, 1, 1, 1, 1},
    //     {1, 0, 1, 1, 1, 1},
    //     {1, 0, 0, 0, 1, 1},
    //     {1, 1, 1, 1, 1, 1},
    //     {1, 1, 1, 1, 1, 1},
    //     {1, 1, 1, 1, 1, 1},
    // };

    // mapf::Grid grid(&free_cells, rows, cols);

    // mapf::Cell* start = grid.getCellPtr(4, 3);
    // mapf::Cell* goal = grid.getCellPtr(1, 2);

    // mapf::AStarSolver aStarSolver;
    // std::list<mapf::Cell*> path = aStarSolver.solve(grid, start, goal);

    // for (mapf::Cell* cell : path) {
    //     mapf::printCell(cell);
    // }

    // std::vector<std::list<mapf::Cell*>>


    // Teste SIPP

    int rows = 9;
    int cols = 9;

    std::vector<std::vector<int>> free_cells = {
        {0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0},
        {1, 1, 1, 1, 1, 1, 1, 1, 1},
        {0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0},
    };

    mapf::Grid grid(&free_cells, rows, cols);
    mapf::AStarSolver aStarSolver;
    mapf::AStarSippSolver sippSolver;

    // info agente 1
    mapf::Cell* start1 = grid.getCellPtr(4, 5);
    mapf::Cell* end1 = grid.getCellPtr(4, 2);
    std::list<mapf::Cell*> path1 = aStarSolver.solve(grid, start1, end1);

    // info agente 2
    mapf::Cell* start2 = grid.getCellPtr(8, 4);
    mapf::Cell* end2 = grid.getCellPtr(0, 4);
    std::list<mapf::Cell*> path2 = aStarSolver.solve(grid, start2, end2);

    // info agente 3
    // o que queremos testar
    std::vector<std::list<mapf::Cell*>> otherAgentPaths = {path1, path2};
    mapf::Cell* start3 = grid.getCellPtr(3, 4);
    mapf::Cell* end3 = grid.getCellPtr(7, 4);
    std::list<mapf::Cell*> path3 = sippSolver.solve(grid, start3, end3, otherAgentPaths);

    for (mapf::Cell* cell : path3) {
        mapf::printCell(cell);
    }

    return 0;
}
