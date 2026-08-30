#pragma once

#include "cell.hpp"

namespace mapf {

    struct EdgeConflict {
        const Cell& cell_1;
        const Cell& cell_2;
        const int time;
    };

}
