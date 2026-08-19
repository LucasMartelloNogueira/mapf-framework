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
    std::unordered_set<std::string> colisions;

    int i = 0;
    for (std::list<mapf::Cell*> path : paths) {

        int t = 0;

        for (mapf::Cell* cell : path) {
            
            std::string verticeAtTime = std::format("{}-{}-{}", cell->position.x, cell->position.y, t);
            // std::printf("verticeTime %s\n", verticeAtTime.c_str());

            if (colisions.contains(verticeAtTime)) {

                std::printf("ja contem o vertice no timesetp %s, i = %d\n", verticeAtTime.c_str(), i);
                return false;
            }

            colisions.insert(verticeAtTime);
            t++;
        }
        i++;
    }

    return true;
}