#pragma once

#include <list>
#include <vector>

#include "mapf/core/instance.hpp"
#include "mapf/core/result.hpp"

namespace mapf {
    class PriorityPlanningSolver {
        public:
            const Instance& instance;
            Result result;

            explicit PriorityPlanningSolver(const Instance& instance);

            // A permutation of instance agent IDs (the indexes 0..n-1).
            Result solve(std::list<int> agentsOrder);

            const std::vector<std::list<Cell*>>& getInitialPaths() const;
            const std::vector<std::list<Cell*>>& getPaths() const;

        private:
            std::vector<std::list<Cell*>> initialPaths;
            std::vector<std::list<Cell*>> paths;
    };
}
