#pragma once

#include "mapf/core/cell.hpp"

namespace mapf {

    struct EdgeKey {
        Cell* from;
        Cell* to;

        bool operator==(const EdgeKey& other) const;
    };

}
