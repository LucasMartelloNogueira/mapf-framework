#pragma once

#include "cell.hpp"

namespace mapf {

    struct AStarNode {
        Cell* cell;
        Cell* parent;
        int g;
        int h;
        int f;
    };

}   