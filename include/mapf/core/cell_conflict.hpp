#pragma once

#include "cell.hpp"

namespace mapf {

    struct CellConflict {
        const Cell& cell;
        const int time;
    };

}
