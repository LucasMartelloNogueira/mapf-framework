#pragma once

#include <list>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "mapf/core/cell.hpp"
#include "mapf/core/result.hpp"
#include "mapf/core/solution_conflicts.hpp"
#include "mapf/pathfinding/sipp/safe_interval_table.hpp"

namespace mapf {

    // vertexAgents: vai servir para identifcar potenciais conflitos
    using VertexAgents = std::unordered_map<Cell*, std::unordered_set<int>>;

    // GoalReservations: serve para identificar o momento que celulas que estao
    //                   permamentemente ocupados por agentes que chegaram nos destinos
    using GoalReservations = std::unordered_map<Cell*, int>;

    struct PathReservationState {
        SafeIntervalTable safeIntervalTable;
        VertexAgents vertex_agents;
        GoalReservations goal_reservations;
    };

    struct LocalPathRepairResult {
        Result metrics;
        std::vector<std::list<Cell*>> initialPaths;
        std::vector<std::list<Cell*>> paths;
        std::vector<int> pathCosts;
        SolutionConflicts initialConflicts;
        SolutionConflicts remainingConflicts;
        PathReservationState reservations;
    };

}
