#include "include/mapf/utils.hpp"


void printPath(std::list<mapf::Cell*> path) {
    for (mapf::Cell* cell : path) {
        mapf::printCell(cell);
    }
}
