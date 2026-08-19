#include <iostream>
#include <list>
#include <vector>

#include "mapf/core/grid.hpp"
#include "mapf/core/cell.hpp"
#include "mapf/pathfinding/a_star.hpp"
#include "mapf/pathfinding/a_star_sipp.hpp"
#include "mapf/utils.hpp"


int main() {
    
    // Teste SIPP

    int rows = 9;
    int cols = 9;

    // A matriz e escrita de cima para baixo; getCellPtr(x, y) usa y cartesiano.
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
    std::vector<std::list<mapf::Cell*>> otherAgentPaths;

    // info agente 1
    mapf::Cell* start1 = grid.getCellPtr(5, 4);
    mapf::Cell* end1 = grid.getCellPtr(2, 4);
    std::list<mapf::Cell*> path1 = sippSolver.solve(grid, start1, end1, otherAgentPaths);
    otherAgentPaths.push_back(path1);

    std::printf("caminho 1:\n");
    printPath(path1);

    // info agente 2
    mapf::Cell* start2 = grid.getCellPtr(4, 1);
    mapf::Cell* end2 = grid.getCellPtr(4, 7);
    std::list<mapf::Cell*> path2 = sippSolver.solve(grid, start2, end2, otherAgentPaths);
    otherAgentPaths.push_back(path2);

    std::printf("caminho 2:\n");
    printPath(path2);

    // info agente 3
    // o que queremos testar
    mapf::Cell* start3 = grid.getCellPtr(4, 5);
    mapf::Cell* end3 = grid.getCellPtr(4, 1);
    std::list<mapf::Cell*> path3 = sippSolver.solve(grid, start3, end3, otherAgentPaths);
    otherAgentPaths.push_back(path3);

    std::printf("caminho 3:\n");
    printPath(path3);

    bool isValidSolution = validateSolution(otherAgentPaths);
    std::printf("is valid solution = %s\n", isValidSolution ? "true" : "false");


    // teste validação para conflitos de aresta
    // agente 1
    mapf::Cell* start4 = grid.getCellPtr(5, 4);
    mapf::Cell* end4 = grid.getCellPtr(2, 4);
    std::list<mapf::Cell*> path4 = aStarSolver.solve(grid, start4, end4);

    std::printf("caminho 4:\n");
    printPath(path4);

    mapf::Cell* start5 = grid.getCellPtr(4, 4);
    mapf::Cell* end5 = grid.getCellPtr(7, 4);
    std::list<mapf::Cell*> path5 = aStarSolver.solve(grid, start5, end5);

    std::printf("caminho 5:\n");
    printPath(path5);
    
    std::vector<std::list<mapf::Cell*>> paths = {path4, path5};
    bool isValidSolution2 = validateSolution(paths);
    std::printf("is valid solution = %s\n", isValidSolution2 ? "true" : "false");
    
    return 0;
}
