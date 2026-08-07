#pragma once

#include "position.hpp"

namespace mapf {

    struct Cell {
        Position position;
        bool isFree;
    };

    void printCell(Cell* cell) {
        std::cout << "(" << cell->position.x << ", " << cell->position.y << ")" << std::endl;
    }

}