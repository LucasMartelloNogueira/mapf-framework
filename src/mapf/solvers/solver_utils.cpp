#include "solver_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mapf::solver_utils {
    namespace {
        // Keep the original nodes so rollback can restore erased entries without
        // allocating. Only entries touched by the old or replacement path are copied.
        template<class Map>
        class ReservationMapUndo {
        public:
            explicit ReservationMapUndo(Map& target) : target(target) {}
            ReservationMapUndo(const ReservationMapUndo&) = delete;
            ReservationMapUndo& operator=(const ReservationMapUndo&) = delete;

            ~ReservationMapUndo() { rollback(); }

            void remember(const typename Map::key_type& key) {
                auto [saved, inserted] = originals.try_emplace(key);
                if (!inserted) {
                    return;
                }
                const auto entry = target.find(key);
                if (entry != target.end()) {
                    // Record ownership before copying: even a failed allocation
                    // leaves the original entry available for rollback.
                    saved->second = target.extract(entry);
                    target.emplace(saved->second.key(), saved->second.mapped());
                }
                // An empty saved node records that this key did not exist.
            }

            void rollback() noexcept {
                if (!active) {
                    return;
                }
                // Erase ALL tentative entries before restoring any originals.
                // The maps never shrink/rehash explicitly during a transaction,
                // so their buckets still fit the original size. Node insertion
                // needs no allocation; these pointer/edge hashers do not throw.
                for (const auto& [key, original] : originals) {
                    target.erase(key);
                }
                for (auto& [key, original] : originals) {
                    if (!original.empty()) {
                        target.insert(std::move(original));
                    }
                }
                active = false;
            }

            void commit() noexcept { active = false; }

        private:
            Map& target;
            std::unordered_map<typename Map::key_type, typename Map::node_type,
                typename Map::hasher, typename Map::key_equal> originals;
            bool active = true;
        };

        int pathCost(const std::list<Cell*>& path) {
            if (path.empty()) {
                return 0;
            }

            return static_cast<int>(path.size()) - 1;
        }

        double elapsedSeconds(std::chrono::steady_clock::time_point startedAt) {
            return std::chrono::duration<double>(
                std::chrono::steady_clock::now() - startedAt
            ).count();
        }

        void mergeBlockedIntervals(std::vector<Interval>& intervals) {
            if (intervals.empty()) {
                return;
            }

            std::sort(
                intervals.begin(),
                intervals.end(),
                [](const Interval& first, const Interval& second) {
                    if (first.start != second.start) {
                        return first.start < second.start;
                    }

                    return first.end < second.end;
                }
            );

            std::vector<Interval> merged;
            merged.push_back(intervals.front());

            for (std::size_t i = 1; i < intervals.size(); i++) {
                Interval& previous = merged.back();
                const Interval& current = intervals[i];

                if (previous.end >= SAFE_INTERVAL_INFINITY || current.start <= previous.end + 1) {
                    previous.end = std::max(previous.end, current.end);
                } else {
                    merged.push_back(current);
                }
            }

            intervals = std::move(merged);
        }

        void validateReservationPath(const std::list<Cell*>& path) {
            if (path.size() > static_cast<std::size_t>(SAFE_INTERVAL_INFINITY) ||
                std::find(path.begin(), path.end(), nullptr) != path.end()) {
                throw std::invalid_argument("Null reservation cell or unsupported SIPP time.");
            }
        }

        const std::unordered_set<int>* occupantsAt(
            const VertexOccupants& vertices, Cell* cell, int time
        ) {
            const auto vertex = vertices.find(cell);
            if (vertex == vertices.end()) {
                return nullptr;
            }
            const auto occupants = vertex->second.find(time);
            return occupants == vertex->second.end() ? nullptr : &occupants->second;
        }

        bool hasRemainingMovement(
            const VertexOccupants& vertices, Cell* from, Cell* to, int arrivalTime
        ) {
            const auto* first = occupantsAt(vertices, from, arrivalTime - 1);
            const auto* second = occupantsAt(vertices, to, arrivalTime);
            if (first == nullptr || second == nullptr) {
                return false;
            }
            if (first->size() > second->size()) {
                std::swap(first, second);
            }
            return std::any_of(first->begin(), first->end(), [second](int agentId) {
                return second->contains(agentId);
            });
        }

        void registerPathReservations(
            PathReservationState& state, const std::list<Cell*>& path, int agentId
        ) {
            if (path.empty()) {
                return;
            }
            if (!state.goal_reservations.emplace(path.back(), GoalReservation{
                    agentId, static_cast<int>(path.size()) - 1}).second) {
                throw std::logic_error("The path's destination is already reserved.");
            }
            Cell* previous = nullptr;
            int time = 0;
            for (Cell* cell : path) {
                state.vertex_agents[cell][time].insert(agentId);
                if (previous != nullptr && previous != cell) {
                    state.safeIntervalTable.blockedEdgeArrivals[{cell, previous}].insert(time);
                }
                previous = cell;
                ++time;
            }
        }

        void rebuildCellSafeIntervals(PathReservationState& state, Cell* cell) {
            std::vector<Interval> blockedIntervals;
            const auto vertex = state.vertex_agents.find(cell);
            if (vertex != state.vertex_agents.end()) {
                blockedIntervals.reserve(vertex->second.size() + 1);
                for (const auto& [time, occupants] : vertex->second) {
                    if (!occupants.empty()) {
                        blockedIntervals.push_back({time, time});
                    }
                }
            }
            const auto goal = state.goal_reservations.find(cell);
            if (goal != state.goal_reservations.end()) {
                blockedIntervals.push_back({goal->second.arrivalTime, SAFE_INTERVAL_INFINITY});
            }
            mergeBlockedIntervals(blockedIntervals);

            std::vector<Interval> safeIntervals;
            int safeStart = 0;
            for (const Interval& blocked : blockedIntervals) {
                if (safeStart < blocked.start) {
                    safeIntervals.push_back({safeStart, blocked.start - 1});
                }
                if (blocked.end >= SAFE_INTERVAL_INFINITY) {
                    safeStart = SAFE_INTERVAL_INFINITY;
                    break;
                }
                safeStart = blocked.end + 1;
            }
            if (safeStart < SAFE_INTERVAL_INFINITY) {
                safeIntervals.push_back({safeStart, SAFE_INTERVAL_INFINITY});
            }
            state.safeIntervalTable.safeIntervalsByCell.insert_or_assign(cell, std::move(safeIntervals));
        }

        double calculateInjustice(
            const std::vector<std::list<Cell*>>& initialPaths,
            const std::vector<std::list<Cell*>>& finalPaths
        ) {
            std::vector<double> differences;
            differences.reserve(std::min(initialPaths.size(), finalPaths.size()));
            double sum = 0.0;

            const std::size_t pathCount = std::min(initialPaths.size(), finalPaths.size());
            for (std::size_t i = 0; i < pathCount; i++) {
                if (initialPaths[i].empty() || finalPaths[i].empty()) {
                    continue;
                }

                const double difference =
                    static_cast<double>(pathCost(finalPaths[i]) - pathCost(initialPaths[i]));
                differences.push_back(difference);
                sum += difference;
            }

            if (differences.empty()) {
                return 0.0;
            }

            const double mean = sum / static_cast<double>(differences.size());
            double squaredDistanceSum = 0.0;
            for (double difference : differences) {
                const double distance = difference - mean;
                squaredDistanceSum += distance * distance;
            }

            return std::sqrt(
                squaredDistanceSum / static_cast<double>(differences.size())
            );
        }
    }

    struct ReservationTransaction::Impl {
    public:
        explicit Impl(PathReservationState& state) :
            state(state), intervals(state.safeIntervalTable.safeIntervalsByCell),
            vertices(state.vertex_agents), edges(state.safeIntervalTable.blockedEdgeArrivals),
            goals(state.goal_reservations) {}

        void removePath(const std::list<Cell*>& path, int agentId) {
            rememberPath(path);
            repairSafeIntervalTable(state, path, agentId);
        }

        void insertPath(const std::list<Cell*>& path, int agentId) {
            rememberPath(path);
            updateReservationState(state, path, agentId);
        }

        void rollback() noexcept {
            intervals.rollback();
            vertices.rollback();
            edges.rollback();
            goals.rollback();
        }

        void commit() noexcept {
            intervals.commit();
            vertices.commit();
            edges.commit();
            goals.commit();
        }

    private:
        void rememberPath(const std::list<Cell*>& path) {
            Cell* previous = nullptr;
            for (Cell* cell : path) {
                intervals.remember(cell);
                vertices.remember(cell);
                if (previous != nullptr && previous != cell) {
                    edges.remember({cell, previous});
                }
                previous = cell;
            }
            if (!path.empty()) {
                goals.remember(path.back());
            }
        }

        PathReservationState& state;
        ReservationMapUndo<SafeIntervalsByCell> intervals;
        ReservationMapUndo<VertexOccupants> vertices;
        ReservationMapUndo<BlockedEdgeArrivals> edges;
        ReservationMapUndo<decltype(PathReservationState::goal_reservations)> goals;
    };

    ReservationTransaction::ReservationTransaction(PathReservationState& state) :
        impl(std::make_unique<Impl>(state)) {}

    ReservationTransaction::~ReservationTransaction() = default;

    void ReservationTransaction::removePath(const std::list<Cell*>& path, int agentId) {
        impl->removePath(path, agentId);
    }

    void ReservationTransaction::insertPath(const std::list<Cell*>& path, int agentId) {
        impl->insertPath(path, agentId);
    }

    void ReservationTransaction::rollback() noexcept { impl->rollback(); }
    void ReservationTransaction::commit() noexcept { impl->commit(); }

    struct PathTransaction::Impl {
        explicit Impl(std::vector<std::list<Cell*>>& paths) : paths(paths) {}

        std::vector<std::list<Cell*>>& paths;
        std::unordered_map<std::size_t, std::list<Cell*>> originals;
        bool active = true;
    };

    PathTransaction::PathTransaction(std::vector<std::list<Cell*>>& paths) :
        impl(std::make_unique<Impl>(paths)) {}

    PathTransaction::~PathTransaction() { rollback(); }

    void PathTransaction::replacePath(std::size_t index, std::list<Cell*>&& replacement) {
        auto& path = impl->paths.at(index);
        // Allocate the undo entry before modifying a path. Swapping these lists
        // transfers their nodes without copying cells or allocating list nodes.
        auto [saved, inserted] = impl->originals.try_emplace(index);
        if (inserted) {
            saved->second.swap(path);
        }
        path.swap(replacement);
    }

    void PathTransaction::rollback() noexcept {
        if (!impl->active) {
            return;
        }
        for (auto& [index, original] : impl->originals) {
            impl->paths[index].swap(original);
        }
        impl->active = false;
    }

    void PathTransaction::commit() noexcept { impl->active = false; }

    void updateResultMetrics(
        LocalPathRepairResult& result,
        const std::vector<Agent>& agents,
        bool requestedSuccess,
        std::chrono::steady_clock::time_point startedAt
    ) {
        result.pathCosts.assign(result.paths.size(), 0);
        int sumOfCosts = 0;
        int makespan = 0;
        bool allPathsExist = result.paths.size() == agents.size();

        for (std::size_t i = 0; i < result.paths.size(); i++) {
            if (result.paths[i].empty()) {
                allPathsExist = false;
            }

            result.pathCosts[i] = pathCost(result.paths[i]);
            sumOfCosts += result.pathCosts[i];
            makespan = std::max(makespan, result.pathCosts[i]);
        }

        const bool conflictFree = result.remainingConflicts.empty();
        const bool success =
            requestedSuccess && allPathsExist && conflictFree;
        const std::size_t numInitialConflicts = result.initialConflicts.vertexEvents.size() +
            result.initialConflicts.edgeEvents.size();
        const std::size_t numUnresolvedConflicts = result.remainingConflicts.vertexEvents.size() +
            result.remainingConflicts.edgeEvents.size();
        const std::size_t numResolvedConflicts = numInitialConflicts -
            std::min(numInitialConflicts, numUnresolvedConflicts);

        result.metrics = Result(
            success,
            sumOfCosts,
            makespan,
            calculateInjustice(result.initialPaths, result.paths),
            elapsedSeconds(startedAt),
            numInitialConflicts,
            numResolvedConflicts,
            numUnresolvedConflicts
        );
    }

    PathReservationState buildReservationState(
        Grid& grid,
        const std::vector<Agent>& agents,
        const std::vector<std::list<Cell*>>& paths,
        std::optional<std::size_t> excludedPathIndex
    ) {
        if (agents.size() != paths.size() || (excludedPathIndex && *excludedPathIndex >= paths.size())) {
            throw std::invalid_argument("Reservation paths and agent indexes must be aligned.");
        }
        PathReservationState state;
        for (std::size_t i = 0; i < paths.size(); ++i) {
            if (excludedPathIndex && *excludedPathIndex == i) {
                continue;
            }
            validateReservationPath(paths[i]);
            registerPathReservations(state, paths[i], agents[i].id);
        }
        state.safeIntervalTable.safeIntervalsByCell.reserve(grid.getCells().size());
        for (Cell& cell : grid.getCells()) {
            rebuildCellSafeIntervals(state, &cell);
        }
        return state;
    }

    void repairSafeIntervalTable(
        PathReservationState& state, const std::list<Cell*>& oldPath, int agentId
    ) {
        validateReservationPath(oldPath);
        if (oldPath.empty()) {
            return;
        }
        const auto goal = state.goal_reservations.find(oldPath.back());
        if (goal == state.goal_reservations.end() || goal->second.agentId != agentId ||
            goal->second.arrivalTime != static_cast<int>(oldPath.size()) - 1) {
            throw std::logic_error("The path's goal reservation does not match its owner and arrival.");
        }
        // Validate every membership before changing any entry. The caller must
        // protect later mutations with an undo log or a disposable state copy.
        std::unordered_set<Cell*> affectedCells;
        int time = 0;
        for (Cell* cell : oldPath) {
            const auto* occupants = occupantsAt(state.vertex_agents, cell, time);
            if (occupants == nullptr || !occupants->contains(agentId)) {
                throw std::logic_error("The path's explicit reservation is missing its owner.");
            }
            affectedCells.insert(cell);
            ++time;
        }
        time = 0;
        for (Cell* cell : oldPath) {
            const auto vertex = state.vertex_agents.find(cell);
            const auto occupants = vertex->second.find(time);
            occupants->second.erase(agentId);
            if (occupants->second.empty()) {
                vertex->second.erase(occupants);
            }
            if (vertex->second.empty()) {
                state.vertex_agents.erase(vertex);
            }
            ++time;
        }
        state.goal_reservations.erase(goal);

        // Endpoint intersections must see the state AFTER all explicit removals.
        Cell* previous = nullptr;
        time = 0;
        auto& blockedEdges = state.safeIntervalTable.blockedEdgeArrivals;
        for (Cell* cell : oldPath) {
            if (previous != nullptr && previous != cell &&
                !hasRemainingMovement(state.vertex_agents, previous, cell, time)) {
                const auto edge = blockedEdges.find({cell, previous});
                if (edge != blockedEdges.end()) {
                    edge->second.erase(time);
                    if (edge->second.empty()) {
                        blockedEdges.erase(edge);
                    }
                }
            }
            previous = cell;
            ++time;
        }
        for (Cell* cell : affectedCells) {
            rebuildCellSafeIntervals(state, cell);
        }
    }

    void updateReservationState(
        PathReservationState& state, const std::list<Cell*>& newPath, int agentId
    ) {
        validateReservationPath(newPath);
        if (newPath.empty()) {
            return;
        }
        if (state.goal_reservations.contains(newPath.back())) {
            throw std::logic_error("The path's destination is already reserved.");
        }
        std::unordered_set<Cell*> affectedCells;
        int time = 0;
        for (Cell* cell : newPath) {
            const auto* occupants = occupantsAt(state.vertex_agents, cell, time);
            if (occupants != nullptr && occupants->contains(agentId)) {
                throw std::logic_error("Remove the agent's old path before registering its replacement.");
            }
            affectedCells.insert(cell);
            ++time;
        }
        registerPathReservations(state, newPath, agentId);
        for (Cell* cell : affectedCells) {
            rebuildCellSafeIntervals(state, cell);
        }
    }

}
