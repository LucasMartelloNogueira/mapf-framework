#pragma once

#include <cstddef>
#include "mapf/core/instance.hpp"
#include "mapf/solvers/local_path_repair_solver.hpp"

namespace mapf {

    class LocalPathRepairParallelSolver {
        private:
            const Instance& instance;
            std::size_t numberOfThreads;
            bool continueIfFailed;
            LocalRepairStrategy localRepairStrategy;

        public:
            LocalPathRepairParallelSolver(
                const Instance& instance,
                std::size_t numberOfThreads,
                bool continueIfFailed = false,
                LocalRepairStrategy localRepairStrategy = LocalRepairStrategy::RESOLVE_BY_AGENT
            );

            LocalPathRepairResult solve();
    };

}
