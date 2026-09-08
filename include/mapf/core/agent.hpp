#pragma once

#include "position.hpp"

namespace mapf {

    struct Agent {
        int id;
        int scenarioId = -1;
        Position currentPosition;
        Position startPosition;
        Position goalPosition; 
    };

}
