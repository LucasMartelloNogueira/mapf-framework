#include "mapf/core/cell.hpp"

#include <iostream>

namespace mapf {

    void printCell(Cell* cell) {
        std::cout << "(" << cell->position.x << ", " << cell->position.y << ")" << std::endl;
    }

}
