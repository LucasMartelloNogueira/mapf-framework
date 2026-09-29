#pragma once

#include "cell.hpp"

namespace mapf {

    struct CellConflict {
        Cell* cell;
        int time;
    };

}
