#pragma once

#include "mapf/core/instance.hpp"
#include "mapf/solvers/local_path_repair_solver.hpp"

namespace mapf {

    class LocalPathRepairIterativeSolver {
        private:
            const Instance& instance;
            bool continueIfFailed;
            LocalRepairStrategy localRepairStrategy;

        public:
            explicit LocalPathRepairIterativeSolver(
                const Instance& instance,
                bool continueIfFailed = false,
                LocalRepairStrategy localRepairStrategy = LocalRepairStrategy::RESOLVE_BY_AGENT
            );

            LocalPathRepairResult solve();
    };

}
