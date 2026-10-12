#pragma once

#include "position.hpp"

namespace mapf {

    struct Agent {
        // Instance assigns this ID to the agent's index in getAgents().
        int id;
        int scenarioId = -1;
        Position currentPosition;
        Position startPosition;
        Position goalPosition; 
    };

}
