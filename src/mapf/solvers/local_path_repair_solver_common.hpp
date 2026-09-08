#pragma once

#include "mapf/core/instance.hpp"
#include "mapf/solvers/local_path_repair_solver.hpp"

#include <chrono>
#include <list>
#include <vector>

namespace mapf::local_path_repair_detail {

    LocalPathRepairResult repairInitialPaths(
        const Instance& instance,
        std::vector<std::list<Cell*>> initialPaths,
        bool continueIfFailed,
        std::chrono::steady_clock::time_point startedAt
    );

}
