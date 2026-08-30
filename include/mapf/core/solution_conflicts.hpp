#pragma once

#include <vector>

#include "cell_conflict.hpp"
#include "edge_conflict.hpp"

namespace mapf {

    struct SolutionConflicts {
        std::vector<CellConflict> cellConflicts;
        std::vector<EdgeConflict> edgeConflicts;
    };

}
