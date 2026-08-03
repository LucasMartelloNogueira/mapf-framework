#pragma once

#include "position.hpp"

namespace mapf {

    struct Cell {
        Position position;
        bool isFree;
    };

}