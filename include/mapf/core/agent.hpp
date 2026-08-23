#pragma once

#include "position.hpp"

namespace mapf {

    struct Agent {
        int id;
        Position currentPosition;
        Position startPosition;
        Position goalPosition; 
    };

}
