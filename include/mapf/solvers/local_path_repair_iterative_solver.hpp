#pragma once

#include "mapf/core/instance.hpp"
#include "mapf/solvers/local_path_repair_solver.hpp"

namespace mapf {

    class LocalPathRepairIterativeSolver {
        private:
            const Instance& instance;
            LocalRepairStrategy localRepairStrategy;
            int maxNewConflicts;

        public:
            explicit LocalPathRepairIterativeSolver(
                const Instance& instance,
                LocalRepairStrategy localRepairStrategy = LocalRepairStrategy::RESOLVE_BY_AGENT,
                int maxNewConflicts = 0
            );

            LocalPathRepairResult solve();
    };

}
