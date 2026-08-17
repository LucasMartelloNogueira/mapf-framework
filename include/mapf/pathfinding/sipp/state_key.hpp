#pragma once

#include "mapf/core/cell.hpp"

namespace mapf {

    struct StateKey {
        Cell* cell;
        int intervalIndex;

        bool operator==(const StateKey& other) const;
    };

}
