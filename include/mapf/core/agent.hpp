#include <string>
#include "position.hpp"

#pragma once

namespace mapf {

    struct Agent {
        std::string id;
        Position currentPosition;
        Position startPosition;
        Position goalPosition; 
    };

}