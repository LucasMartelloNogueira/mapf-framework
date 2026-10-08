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

    enum class LocalRepairStrategy {
        RESOLVE_BY_AGENT,
        RESOLVE_BY_TIME
    };

    // Occupants are Agent::id values, equal to path indexes for instance agents.
    using VertexOccupants = std::unordered_map<Cell*,
        std::unordered_map<int, std::unordered_set<int>>>;

    struct GoalReservation {
        int agentId;
        int arrivalTime;
    };

    struct PathReservationState {
        SafeIntervalTable safeIntervalTable;
        VertexOccupants vertex_agents;
        std::unordered_map<Cell*, GoalReservation> goal_reservations;
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
