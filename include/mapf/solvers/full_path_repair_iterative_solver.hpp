#pragma once

#include "mapf/core/instance.hpp"
#include "mapf/solvers/local_path_repair_solver.hpp"

namespace mapf {

    class FullPathRepairIterativeSolver {
        private:
            const Instance& instance;

        public:
            explicit FullPathRepairIterativeSolver(const Instance& instance);

            // The instance's grid must outlive the returned non-owning paths.
            LocalPathRepairResult solve();
    };

}
