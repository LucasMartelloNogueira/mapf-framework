#include <iostream>
#include <list>
#include <vector>

#include "mapf/core/grid.hpp"
#include "mapf/core/cell.hpp"
#include "mapf/pathfinding/a_star.hpp"
#include "mapf/pathfinding/a_star_sipp.hpp"
#include "mapf/utils.hpp"
#include "mapf/core/agent.hpp"
#include "mapf/core/position.hpp"
#include "mapf/core/instance.hpp"
#include "mapf/solvers/local_path_repair_parallel_solver.hpp"
#include "mapf/core/cell_conflict.hpp"
#include "mapf/core/edge_conflict.hpp"


int main() {
    
    // Teste SIPP

    // int rows = 9;
    // int cols = 9;

    // // A matriz e escrita de cima para baixo; getCellPtr(x, y) usa y cartesiano.
    // std::vector<std::vector<int>> free_cells = {
    //     {0, 0, 0, 0, 1, 0, 0, 0, 0},
    //     {0, 0, 0, 0, 1, 0, 0, 0, 0},
    //     {0, 0, 0, 0, 1, 0, 0, 0, 0},
    //     {0, 0, 0, 0, 1, 0, 0, 0, 0},
    //     {1, 1, 1, 1, 1, 1, 1, 1, 1},
    //     {0, 0, 0, 0, 1, 0, 0, 0, 0},
    //     {0, 0, 0, 0, 1, 0, 0, 0, 0},
    //     {0, 0, 0, 0, 1, 0, 0, 0, 0},
    //     {0, 0, 0, 0, 1, 0, 0, 0, 0},
    // };

    // mapf::Grid grid(&free_cells, rows, cols);
    // mapf::AStarSolver aStarSolver;
    // mapf::AStarSippSolver sippSolver;
    // std::vector<std::list<mapf::Cell*>> otherAgentPaths;

    // // info agente 1
    // mapf::Cell* start1 = grid.getCellPtr(5, 4);
    // mapf::Cell* end1 = grid.getCellPtr(2, 4);
    // std::list<mapf::Cell*> path1 = sippSolver.solve(grid, start1, end1, otherAgentPaths);
    // otherAgentPaths.push_back(path1);

    // std::printf("caminho 1:\n");
    // printPath(path1);

    // // info agente 2
    // mapf::Cell* start2 = grid.getCellPtr(4, 1);
    // mapf::Cell* end2 = grid.getCellPtr(4, 7);
    // std::list<mapf::Cell*> path2 = sippSolver.solve(grid, start2, end2, otherAgentPaths);
    // otherAgentPaths.push_back(path2);

    // std::printf("caminho 2:\n");
    // printPath(path2);

    // // info agente 3
    // // o que queremos testar
    // mapf::Cell* start3 = grid.getCellPtr(4, 5);
    // mapf::Cell* end3 = grid.getCellPtr(4, 1);
    // std::list<mapf::Cell*> path3 = sippSolver.solve(grid, start3, end3, otherAgentPaths);
    // otherAgentPaths.push_back(path3);

    // std::printf("caminho 3:\n");
    // printPath(path3);

    // bool isValidSolution = validateSolution(otherAgentPaths);
    // std::printf("is valid solution = %s\n", isValidSolution ? "true" : "false");


    // // teste validação para conflitos de aresta
    // // agente 1
    // mapf::Cell* start4 = grid.getCellPtr(5, 4);
    // mapf::Cell* end4 = grid.getCellPtr(2, 4);
    // std::list<mapf::Cell*> path4 = aStarSolver.solve(grid, start4, end4);

    // std::printf("caminho 4:\n");
    // printPath(path4);

    // mapf::Cell* start5 = grid.getCellPtr(4, 4);
    // mapf::Cell* end5 = grid.getCellPtr(7, 4);
    // std::list<mapf::Cell*> path5 = aStarSolver.solve(grid, start5, end5);

    // std::printf("caminho 5:\n");
    // printPath(path5);
    
    // std::vector<std::list<mapf::Cell*>> paths = {path4, path5};
    // bool isValidSolution2 = validateSolution(paths);
    // std::printf("is valid solution = %s\n", isValidSolution2 ? "true" : "false");

    // teste de validação de conflito quando agente fica parado no destino

    std::vector<std::vector<int>> free_cells = {
        {1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1},
    };

    int rows = free_cells.size();
    int cols = free_cells[0].size();

    mapf::Grid grid(&free_cells, rows, cols);

    mapf::Agent a1 = {
        .id = 0,
        .currentPosition = mapf::Position{.x = 1, .y = 1},
        .startPosition = mapf::Position{.x = 1, .y = 1},
        .goalPosition = mapf::Position{.x = 1, .y = 3},
    };

    mapf::Agent a2 = {
        .id = 1,
        .currentPosition = mapf::Position{.x = 0, .y = 2},
        .startPosition = mapf::Position{.x = 0, .y = 2},
        .goalPosition = mapf::Position{.x = 2, .y = 2},
    };


    std::vector<mapf::Agent> agents = {a1, a2};
    mapf::Instance instance(
        &free_cells,
        rows,
        cols,
        agents
    );

    mapf::LocalPathRepairParallelSolver solver(instance, 2);
    mapf::LocalPathRepairResult result = solver.solve();

    int conflict_i = 0;
    std::printf("conflitos iniciais\n");
    for (mapf::CellConflict conflict : result.initialConflicts.cellConflicts) {
        std::printf("[%d] conflict: %d - %d no tempo %d\n", conflict_i, conflict.cell.position.x, conflict.cell.position.y, conflict.time);
    }

    int i = 0;
    for (std::list<mapf::Cell*> path : result.paths) {
        std::printf("caminho %i\n", i);
        printPath(path);
        i++;
    }


    // std::printf("rows: %d\n", rows);
    // std::printf("cols: %d\n", cols);

    // mapf::Grid grid(&free_cells, rows, cols);
    // // mapf::AStarSippSolver sippSolver;
    // mapf::AStarSolver aStarSolver;
    // std::vector<std::list<mapf::Cell*>> paths;
    // // agente 1

    // mapf::Cell* start5 = grid.getCellPtr(4, 0);
    // mapf::Cell* end5 = grid.getCellPtr(4, 2);
    // // std::list<mapf::Cell*> path5 = sippSolver.solve(grid, start5, end5, paths);
    // std::list<mapf::Cell*> path5 = aStarSolver.solve(grid, start5, end5);
    // paths.push_back(path5);

    // std::printf("path agente 1\n");
    // printPath(path5);


    // mapf::Cell* start6 = grid.getCellPtr(0, 2);
    // mapf::Cell* end6 = grid.getCellPtr(8, 2);
    // // std::list<mapf::Cell*> path6 = sippSolver.solve(grid, start6, end6, paths);
    // std::list<mapf::Cell*> path6 = aStarSolver.solve(grid, start6, end6);
    

    // paths.push_back(path6);
    // std::printf("path agente 2\n");
    // printPath(path6);

    // bool isValid = validateSolution(paths);
    // std::printf("is valid: %s\n", isValid ? "true" : "false");

    // return 0;
}
