#pragma once

#include <list>

#include "mapf/core/instance.hpp"
#include "mapf/core/result.hpp"

namespace mapf {
    class PriorityPlanningSolver {
        public:
            const Instance& instance;
            Result result;

            explicit PriorityPlanningSolver(const Instance& instance);

            Result solve(std::list<int> agentsOrder);
    };
}
