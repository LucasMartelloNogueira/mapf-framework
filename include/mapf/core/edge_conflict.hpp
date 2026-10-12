#pragma once

#include "cell.hpp"

namespace mapf {

    struct EdgeConflict {
        Cell* cell_1;
        Cell* cell_2;
        int time;
    };

}
