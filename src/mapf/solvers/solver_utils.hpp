#pragma once

#include "mapf/core/instance.hpp"
#include "mapf/solvers/local_path_repair_solver.hpp"

#include <chrono>
#include <cstddef>
#include <list>
#include <memory>
#include <optional>
#include <vector>

namespace mapf::solver_utils {

    // Tests a complete path or a suffix beginning at absolute time t against
    // reservations that exclude the active agent's old path. Vertex events are
    // counted once per cell/time, edge swaps once per movement, and visits to
    // the final cell after the candidate arrives are conflicts with its
    // permanent goal occupation. Returns false as soon as maxNewConflict events
    // have been found. Structural path validation remains the caller's duty.
    bool canAcceptNewPath(
        const VertexOccupants& vertexOccupants,
        const std::unordered_map<Cell*, GoalReservation>& goalReservations,
        const std::list<Cell*>& newPath,
        int agentId,
        int t,
        int maxNewConflict
    );

    PathReservationState buildReservationState(
        Grid& grid,
        const std::vector<Agent>& agents,
        const std::vector<std::list<Cell*>>& paths,
        std::optional<std::size_t> excludedPathIndex = std::nullopt
    );

    // Complete paths start at time zero. Removal requires the registered path;
    // replacement insertion requires its prior removal. On allocation failure,
    // the caller must restore a transaction or discard this state.
    void repairSafeIntervalTable(
        PathReservationState& state, const std::list<Cell*>& oldPath, int agentId
    );

    void updateReservationState(
        PathReservationState& state, const std::list<Cell*>& newPath, int agentId
    );

    // One transaction may cover several agents. Log individual cell/time/agent
    // memberships, edge times, goals and replaced interval vectors, without
    // copying other reservations in the same cell. Failure/exception replays
    // original nodes in reverse order without allocation.
    // Explicitly rollback before returning a result that owns the state.
    class ReservationTransaction {
    public:
        explicit ReservationTransaction(PathReservationState& state);
        ~ReservationTransaction();
        ReservationTransaction(const ReservationTransaction&) = delete;
        ReservationTransaction& operator=(const ReservationTransaction&) = delete;

        void removePath(const std::list<Cell*>& path, int agentId);
        void insertPath(const std::list<Cell*>& path, int agentId);
        void rollback() noexcept;
        void commit() noexcept;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl;
    };

    // Saves only replaced paths, transferring list nodes instead of copying.
    // Keep the vector size fixed until commit/rollback. Restore before computing
    // failure metrics or returning a result that owns the paths.
    class PathTransaction {
    public:
        explicit PathTransaction(std::vector<std::list<Cell*>>& paths);
        ~PathTransaction();
        PathTransaction(const PathTransaction&) = delete;
        PathTransaction& operator=(const PathTransaction&) = delete;

        void replacePath(std::size_t index, std::list<Cell*>&& replacement);
        void rollback() noexcept;
        void commit() noexcept;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl;
    };

    void updateResultMetrics(
        LocalPathRepairResult& result,
        const std::vector<Agent>& agents,
        bool requestedSuccess,
        std::chrono::steady_clock::time_point startedAt
    );

}
