#pragma once

#include "state_key.hpp"

namespace mapf {

    struct ParentInfo {
        StateKey parent;
        bool hasParent;
    };

}
