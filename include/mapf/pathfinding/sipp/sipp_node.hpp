#pragma once

#include "state_key.hpp"

namespace mapf {

    struct SippNode {
        StateKey key;
        int time;
        int g;
        int h;
        int f;
    };

}
