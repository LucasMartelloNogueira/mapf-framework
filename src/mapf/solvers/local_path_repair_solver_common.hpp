#pragma once

#include "mapf/core/instance.hpp"
#include "mapf/solvers/local_path_repair_solver.hpp"

#include <chrono>
#include <list>
#include <optional>
#include <vector>

namespace mapf::local_path_repair_detail {

    PathReservationState buildReservationState(
        Grid& grid,
        const std::vector<Agent>& agents,
        const std::vector<std::list<Cell*>>& paths,
        std::optional<std::size_t> excludedPathIndex = std::nullopt
    );

    // Mutate a disposable copy. Paths must be complete, with time starting at zero.
    // Removal requires the registered path; insertion requires its prior removal.
    // After an allocation failure, discard the temporary state.
    void repairSafeIntervalTable(
        PathReservationState& state, const std::list<Cell*>& oldPath, int agentId
    );

    void updateReservationState(
        PathReservationState& state, const std::list<Cell*>& newPath, int agentId
    );

    void validateLocalRepairStrategy(LocalRepairStrategy strategy);

    LocalPathRepairResult repairInitialPaths(
        const Instance& instance,
        std::vector<std::list<Cell*>> initialPaths,
        bool continueIfFailed,
        std::chrono::steady_clock::time_point startedAt,
        LocalRepairStrategy localRepairStrategy
    );

}
