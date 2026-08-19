#include <unordered_set>
#include <string>
#include <format>
#include <vector>

#include "mapf/utils.hpp"


void printPath(std::list<mapf::Cell*> path) {
    for (mapf::Cell* cell : path) {
        mapf::printCell(cell);
    }
}

bool validateSolution(std::vector<std::list<mapf::Cell*>> paths) {
    std::unordered_set<std::string> verticeColisions;
    std::unordered_set<std::string> edgeColisions;

    int i = 0;
    for (std::list<mapf::Cell*> path : paths) {

        int t = 0;
        mapf::Cell* previousCell = nullptr;

        for (mapf::Cell* cell : path) {
            
            std::string verticeAtTime = std::format("{}-{}-{}", cell->position.x, cell->position.y, t);
            // std::printf("verticeTime %s\n", verticeAtTime.c_str());

            if (verticeColisions.contains(verticeAtTime)) {

                std::printf("ja contem o vertice no timesetp %s, i = %d\n", verticeAtTime.c_str(), i);
                return false;
            }

            verticeColisions.insert(verticeAtTime);

            // checking edge conflict
            if (t > 0) {
                std::string edge = std::format("({},{})-({},{})-{}", 
                    cell->position.x, cell->position.y, 
                    previousCell->position.x, previousCell->position.y, 
                    t
                );

                std::string invertedEdge = std::format("({},{})-({},{})-{}", 
                    previousCell->position.x, previousCell->position.y, 
                    cell->position.x, cell->position.y, 
                    t
                );

                if (edgeColisions.contains(edge) || edgeColisions.contains(invertedEdge)) {
                    std::printf("aresta %s já foi usada no tempo %d\n", edge.c_str(), t);
                    return false;
                }

                edgeColisions.insert(edge);
            }
            previousCell = cell;
            t++;
        }
        i++;
    }

    return true;
}