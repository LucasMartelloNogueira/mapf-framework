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

    inline constexpr int AGENT_PATH_UNCHANGED = -1;
    inline constexpr int AGENT_FULL_REPLAN = -2;
    inline constexpr int AGENT_REPAIR_FAILED = -3;

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
        // Indexed by agent ID. Nonnegative values are zero-based ordinals of
        // accepted local anchors. Negative values describe an unchanged path,
        // an accepted full replan, or a failed repair using the constants above.
        std::vector<int> agentAnchorsIndex;
        SolutionConflicts initialConflicts;
        SolutionConflicts remainingConflicts;
        PathReservationState reservations;
    };

}
