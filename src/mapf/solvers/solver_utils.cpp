#include "solver_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

namespace mapf::solver_utils {
    namespace {
        // Each entry owns only the removed node, never a copy of its value.
        // Parent containers are extracted only after becoming empty. Reverse
        // replay restores those parents before their child nodes, without allocation.
        template<class Container>
        struct ReservationUndoEntry {
            Container* target;
            typename Container::key_type key;
            typename Container::node_type original;

            void rollback() noexcept {
                target->erase(key);
                if (!original.empty()) target->insert(std::move(original));
            }
        };

        class ReservationUndoLog {
            using Entry = std::variant<
                ReservationUndoEntry<std::unordered_set<int>>,
                ReservationUndoEntry<VertexOccupants::mapped_type>,
                ReservationUndoEntry<VertexOccupants>,
                ReservationUndoEntry<BlockedEdgeArrivals>,
                ReservationUndoEntry<decltype(PathReservationState::goal_reservations)>,
                ReservationUndoEntry<SafeIntervalsByCell>>;
            std::vector<Entry> entries;
            bool active = true;

        public:
            ~ReservationUndoLog() { rollback(); }

            template<class Container>
            void extract(Container& target, const typename Container::key_type& key) {
                // Allocate the undo entry before touching the target. Rehashing
                // does not invalidate references to mapped containers.
                auto& entry = entries.emplace_back(ReservationUndoEntry<Container>{&target, key, {}});
                std::get<ReservationUndoEntry<Container>>(entry).original = target.extract(key);
            }

            void rollback() noexcept {
                if (!active) return;
                for (auto it = entries.rbegin(); it != entries.rend(); ++it) {
                    std::visit([](auto& entry) { entry.rollback(); }, *it);
                }
                active = false;
            }

            void commit() noexcept { active = false; }
        };

        class ReservationEditor {
            ReservationUndoLog* undo;

        public:
            explicit ReservationEditor(ReservationUndoLog* undo) : undo(undo) {}

            template<class Map>
            typename Map::mapped_type& ensure(Map& target, const typename Map::key_type& key) {
                const auto found = target.find(key);
                if (found != target.end()) return found->second;
                if (undo) undo->extract(target, key);
                return target.try_emplace(key).first->second;
            }

            void insert(std::unordered_set<int>& target, int value) {
                if (target.contains(value)) return;
                if (undo) undo->extract(target, value);
                target.insert(value);
            }

            template<class Container>
            void erase(Container& target, const typename Container::key_type& key) {
                if (!target.contains(key)) return;
                if (undo) undo->extract(target, key);
                else target.erase(key);
            }

            template<class Map>
            void replace(Map& target, const typename Map::key_type& key, typename Map::mapped_type value) {
                if (undo) {
                    undo->extract(target, key);
                    target.emplace(key, std::move(value));
                } else {
                    target.insert_or_assign(key, std::move(value));
                }
            }
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
            PathReservationState& state, const std::list<Cell*>& path, int agentId,
            ReservationUndoLog* undo = nullptr
        ) {
            if (path.empty()) {
                return;
            }
            if (state.goal_reservations.contains(path.back())) {
                throw std::logic_error("The path's destination is already reserved.");
            }
            ReservationEditor edit(undo);
            edit.replace(state.goal_reservations, path.back(), {agentId, static_cast<int>(path.size()) - 1});
            Cell* previous = nullptr;
            int time = 0;
            for (Cell* cell : path) {
                auto& times = edit.ensure(state.vertex_agents, cell);
                edit.insert(edit.ensure(times, time), agentId);
                if (previous != nullptr && previous != cell) {
                    edit.insert(edit.ensure(state.safeIntervalTable.blockedEdgeArrivals, {cell, previous}), time);
                }
                previous = cell;
                ++time;
            }
        }

        void rebuildCellSafeIntervals(PathReservationState& state, Cell* cell, ReservationUndoLog* undo = nullptr) {
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
            ReservationEditor edit(undo);
            if (blockedIntervals.empty()) {
                edit.erase(state.safeIntervalTable.safeIntervalsByCell, cell);
                return;
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
            edit.replace(state.safeIntervalTable.safeIntervalsByCell, cell, std::move(safeIntervals));
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

    bool canAcceptNewPath(
        const VertexOccupants& vertexOccupants,
        const std::unordered_map<Cell*, GoalReservation>& goalReservations,
        const std::list<Cell*>& newPath,
        int agentId,
        int t,
        int maxNewConflict
    ) {
        if (t < 0) {
            throw std::invalid_argument("Initial path time must be nonnegative.");
        }
        if (!newPath.empty() && newPath.size() - 1 >
            static_cast<std::size_t>(std::numeric_limits<int>::max() - t)) {
            throw std::invalid_argument("Path time is unrepresentable.");
        }
        if (maxNewConflict <= 0) {
            return false;
        }

        const auto hasOtherAgent = [agentId](const std::unordered_set<int>* occupants) {
            return occupants != nullptr &&
                std::any_of(occupants->begin(), occupants->end(), [agentId](int occupant) {
                    return occupant != agentId;
                });
        };
        const auto hasReverseMovement = [&](Cell* from, Cell* to, int arrivalTime) {
            if (from == to || arrivalTime <= 0) {
                return false;
            }
            const auto* reverseStarts = occupantsAt(
                vertexOccupants, to, arrivalTime - 1);
            const auto* reverseArrives = occupantsAt(
                vertexOccupants, from, arrivalTime);
            if (reverseStarts == nullptr || reverseArrives == nullptr) {
                return false;
            }
            if (reverseStarts->size() > reverseArrives->size()) {
                std::swap(reverseStarts, reverseArrives);
            }
            return std::any_of(reverseStarts->begin(), reverseStarts->end(),
                [&](int occupant) {
                    return occupant != agentId && reverseArrives->contains(occupant);
                });
        };

        int newConflicts = 0;
        const auto reachesLimit = [&]() {
            ++newConflicts;
            return newConflicts >= maxNewConflict;
        };

        Cell* previous = nullptr;
        std::size_t offset = 0;
        for (Cell* cell : newPath) {
            if (cell == nullptr) {
                throw std::invalid_argument("Paths must not contain null cells.");
            }
            const int time = t + static_cast<int>(offset);

            bool vertexConflict = hasOtherAgent(occupantsAt(vertexOccupants, cell, time));
            const auto goal = goalReservations.find(cell);
            if (goal != goalReservations.end() && goal->second.agentId != agentId &&
                goal->second.arrivalTime <= time) {
                vertexConflict = true;
            }
            if (vertexConflict && reachesLimit()) {
                return false;
            }
            if (previous != nullptr && hasReverseMovement(previous, cell, time) && reachesLimit()) {
                return false;
            }

            previous = cell;
            ++offset;
        }

        if (newPath.empty()) {
            return true;
        }

        const int finalArrival = t + static_cast<int>(newPath.size() - 1);
        Cell* finalCell = newPath.back();

        // The candidate remains at its destination after its explicit path
        // ends. Every later explicit visit by another agent creates one vertex
        // event with that permanent occupation.
        const auto finalVisits = vertexOccupants.find(finalCell);
        if (finalVisits != vertexOccupants.end()) {
            for (const auto& [visitTime, occupants] : finalVisits->second) {
                if (visitTime > finalArrival && hasOtherAgent(&occupants) && reachesLimit()) {
                    return false;
                }
            }
        }

        return true;
    }

    namespace {
        void removePathReservations(PathReservationState& state, const std::list<Cell*>& path,
            int agentId, ReservationUndoLog* undo);
        void insertPathReservations(PathReservationState& state, const std::list<Cell*>& path,
            int agentId, ReservationUndoLog* undo);
    }

    struct ReservationTransaction::Impl {
        explicit Impl(PathReservationState& state) : state(state) {}

        void removePath(const std::list<Cell*>& path, int agentId) {
            removePathReservations(state, path, agentId, &undo);
        }
        void insertPath(const std::list<Cell*>& path, int agentId) {
            insertPathReservations(state, path, agentId, &undo);
        }
        void rollback() noexcept { undo.rollback(); }
        void commit() noexcept { undo.commit(); }

        PathReservationState& state;
        ReservationUndoLog undo;
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
        Grid&,
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
        for (const auto& [cell, occupants] : state.vertex_agents) {
            rebuildCellSafeIntervals(state, cell);
        }
        return state;
    }

    namespace {
        void removePathReservations(
            PathReservationState& state, const std::list<Cell*>& oldPath, int agentId, ReservationUndoLog* undo
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
            ReservationEditor edit(undo);
            time = 0;
            for (Cell* cell : oldPath) {
                const auto vertex = state.vertex_agents.find(cell);
                const auto occupants = vertex->second.find(time);
                edit.erase(occupants->second, agentId);
                if (occupants->second.empty()) {
                    edit.erase(vertex->second, time);
                }
                if (vertex->second.empty()) {
                    edit.erase(state.vertex_agents, cell);
                }
                ++time;
            }
            edit.erase(state.goal_reservations, oldPath.back());

            // Endpoint intersections must see the state AFTER all explicit removals.
            Cell* previous = nullptr;
            time = 0;
            auto& blockedEdges = state.safeIntervalTable.blockedEdgeArrivals;
            for (Cell* cell : oldPath) {
                if (previous != nullptr && previous != cell &&
                    !hasRemainingMovement(state.vertex_agents, previous, cell, time)) {
                    const auto edge = blockedEdges.find({cell, previous});
                    if (edge != blockedEdges.end()) {
                        edit.erase(edge->second, time);
                        if (edge->second.empty()) {
                            edit.erase(blockedEdges, EdgeKey{cell, previous});
                        }
                    }
                }
                previous = cell;
                ++time;
            }
            for (Cell* cell : affectedCells) {
                rebuildCellSafeIntervals(state, cell, undo);
            }
        }

        void insertPathReservations(
            PathReservationState& state, const std::list<Cell*>& newPath, int agentId, ReservationUndoLog* undo
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
            registerPathReservations(state, newPath, agentId, undo);
            for (Cell* cell : affectedCells) {
                rebuildCellSafeIntervals(state, cell, undo);
            }
        }
    }

    void repairSafeIntervalTable(PathReservationState& state, const std::list<Cell*>& path, int agentId) {
        removePathReservations(state, path, agentId, nullptr);
    }

    void updateReservationState(PathReservationState& state, const std::list<Cell*>& path, int agentId) {
        insertPathReservations(state, path, agentId, nullptr);
    }

}
