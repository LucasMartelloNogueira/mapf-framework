#pragma once

#include <cstddef>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "mapf/core/cell.hpp"
#include "mapf/core/instance.hpp"
#include "mapf/core/result.hpp"
#include "mapf/core/solution_conflicts.hpp"
#include "mapf/pathfinding/sipp/safe_interval_table.hpp"

namespace mapf {

    using VertexAgents = std::unordered_map<Cell*, std::unordered_set<int>>;
    using GoalReservations = std::unordered_map<Cell*, int>;

    struct PathReservationState {
        SafeIntervalTable safeIntervalTable;
        VertexAgents vertex_agents;
        GoalReservations goal_reservations;
    };

    struct LocalPathRepairResult {
        Result metrics;
        std::vector<std::list<Cell*>> paths;
        std::vector<int> pathCosts;
        SolutionConflicts initialConflicts;
        SolutionConflicts remainingConflicts;
        PathReservationState reservations;
    };

    class LocalPathRepairParallelSolver {
        private:
            const Instance& instance;
            std::size_t numberOfThreads;

        public:
            LocalPathRepairParallelSolver(
                const Instance& instance,
                std::size_t numberOfThreads
            );

            LocalPathRepairResult solve();
    };

}
