#include "local_path_repair_solver_common.hpp"

#include "mapf/pathfinding/a_star_sipp.hpp"
#include "mapf/utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <variant>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mapf {

    namespace {
        struct SelectedConflict {
            std::size_t pathIndex;
            std::variant<CellConflict, EdgeConflict> conflict;
        };

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

        class ReservationTransaction {
        public:
            explicit ReservationTransaction(PathReservationState& state) :
                state(state), intervals(state.safeIntervalTable.safeIntervalsByCell),
                vertices(state.vertex_agents), edges(state.safeIntervalTable.blockedEdgeArrivals),
                goals(state.goal_reservations) {}

            void removePath(const std::list<Cell*>& path, int agentId) {
                rememberPath(path);
                local_path_repair_detail::repairSafeIntervalTable(state, path, agentId);
            }

            void insertPath(const std::list<Cell*>& path, int agentId) {
                rememberPath(path);
                local_path_repair_detail::updateReservationState(state, path, agentId);
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

        bool isValidStep(Cell* from, Cell* to) {
            if (from == nullptr || to == nullptr) {
                return false;
            }

            if (from == to) {
                return true;
            }

            const int distance =
                std::abs(from->position.x - to->position.x) +
                std::abs(from->position.y - to->position.y);
            return distance == 1;
        }

        bool structurallyValid(const std::list<Cell*>& path, Cell* start, Cell* goal) {
            if (path.empty() || path.size() > static_cast<std::size_t>(SAFE_INTERVAL_INFINITY) ||
                path.front() != start || path.back() != goal) {
                return false;
            }
            Cell* previous = nullptr;
            for (Cell* cell : path) {
                if (cell == nullptr || !cell->isFree ||
                    (previous != nullptr && !isValidStep(previous, cell))) {
                    return false;
                }
                previous = cell;
            }
            return true;
        }

        std::string configurationFingerprint(
            const std::vector<std::list<Cell*>>& paths,
            std::size_t replacedIndex,
            const std::list<Cell*>& replacement
        ) {
            std::ostringstream output;
            for (std::size_t i = 0; i < paths.size(); ++i) {
                const auto& path = i == replacedIndex ? replacement : paths[i];
                output << path.size() << ':';
                for (Cell* cell : path) {
                    output << cell->position.x << ',' << cell->position.y << ';';
                }
                output << '|';
            }
            return output.str();
        }

        std::optional<SelectedConflict> selectByAgent(const SolutionConflicts& conflicts) {
            for (std::size_t i = 0; i < conflicts.byAgent.size(); ++i) {
                if (!conflicts.byAgent[i].empty()) {
                    return SelectedConflict{i, conflicts.byAgent[i].front()};
                }
            }
            return std::nullopt;
        }

        std::optional<SelectedConflict> selectByTime(const SolutionConflicts& conflicts) {
            std::optional<SelectedConflict> earliest;
            for (std::size_t i = 0; i < conflicts.byAgent.size(); ++i) {
                const auto& records = conflicts.byAgent[i];
                // Iteration order breaks identical-record ties by path index.
                if (!records.empty() && (!earliest ||
                    conflictRecordComesBefore(records.front(), earliest->conflict))) {
                    earliest = SelectedConflict{i, records.front()};
                }
            }
            return earliest;
        }

        std::vector<std::size_t> adaptiveAnchors(std::size_t initialAnchor) {
            std::vector<std::size_t> anchors;
            std::unordered_set<std::size_t> seen;
            std::size_t anchor = initialAnchor;

            while (seen.insert(anchor).second) {
                anchors.push_back(anchor);
                if (anchor == 0) {
                    break;
                }

                anchor /= 2;
            }

            return anchors;
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

    void local_path_repair_detail::updateResultMetrics(
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

    PathReservationState local_path_repair_detail::buildReservationState(
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

    void local_path_repair_detail::repairSafeIntervalTable(
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

    void local_path_repair_detail::updateReservationState(
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

    void local_path_repair_detail::validateLocalRepairStrategy(LocalRepairStrategy strategy) {
        switch (strategy) {
        case LocalRepairStrategy::RESOLVE_BY_AGENT:
        case LocalRepairStrategy::RESOLVE_BY_TIME:
            return;
        default:
            throw std::invalid_argument("Unknown local repair strategy.");
        }
    }

    LocalPathRepairResult local_path_repair_detail::repairInitialPaths(
        const Instance& instance,
        std::vector<std::list<Cell*>> initialPaths,
        std::chrono::steady_clock::time_point startedAt,
        LocalRepairStrategy localRepairStrategy
    ) {
        validateLocalRepairStrategy(localRepairStrategy);
        const std::vector<Agent>& agents = instance.getAgents();
        if (initialPaths.size() != agents.size()) {
            throw std::invalid_argument("Initial paths must be aligned with instance agents.");
        }
        Grid& grid = const_cast<Grid&>(instance.getGrid());

        LocalPathRepairResult result;
        result.initialPaths = std::move(initialPaths);
        result.paths = result.initialPaths;
        result.initialConflicts = getCollision(result.initialPaths);
        result.remainingConflicts = result.initialConflicts;
        result.reservations = buildReservationState(grid, agents, result.paths);

        AStarSippSolver sipp;
        std::unordered_set<std::string> visitedConfigurations;
        visitedConfigurations.insert(configurationFingerprint(result.paths, result.paths.size(), {}));

        while (true) {
            std::optional<SelectedConflict> selected;
            switch (localRepairStrategy) {
            case LocalRepairStrategy::RESOLVE_BY_AGENT:
                selected = selectByAgent(result.remainingConflicts);
                break;
            case LocalRepairStrategy::RESOLVE_BY_TIME:
                selected = selectByTime(result.remainingConflicts);
                break;
            default:
                throw std::invalid_argument("Unknown local repair strategy.");
            }
            if (!selected) {
                break;
            }

            const std::size_t activeIndex = selected->pathIndex;
            const std::vector<Cell*> oldPath(result.paths[activeIndex].begin(), result.paths[activeIndex].end());
            ReservationTransaction repair(result.reservations);
            repair.removePath(result.paths[activeIndex], agents[activeIndex].id);

            std::list<Cell*> newPath;
            std::string nextConfiguration;
            bool repairSuccess = false;
            int repairStartIndex = 0;
            const int time = conflictTime(selected->conflict);
            if (time > 0 && static_cast<std::size_t>(time) < oldPath.size()) {
                const std::size_t reconnectIndex = static_cast<std::size_t>(time) +
                    (std::holds_alternative<CellConflict>(selected->conflict) ? 1 : 0);
                if (reconnectIndex < oldPath.size()) {
                    const auto goalOccupation = reconnectIndex + 1 == oldPath.size()
                        ? AStarSippSolver::GoalOccupation::Permanent
                        : AStarSippSolver::GoalOccupation::Transient;
                    for (std::size_t anchor : adaptiveAnchors(static_cast<std::size_t>(time - 1))) {
                        const std::list<Cell*> bridge = sipp.solve(grid, oldPath[anchor],
                            oldPath[reconnectIndex], result.reservations.safeIntervalTable,
                            static_cast<int>(anchor), goalOccupation);
                        if (!structurallyValid(bridge, oldPath[anchor], oldPath[reconnectIndex])) {
                            continue;
                        }
                        const std::size_t tailSize = oldPath.size() - reconnectIndex - 1;
                        const std::size_t limit = static_cast<std::size_t>(SAFE_INTERVAL_INFINITY);
                        if (bridge.size() > limit - anchor || tailSize > limit - anchor - bridge.size()) {
                            continue;
                        }

                        // Preserve the prefix and suffix without duplicating the bridge endpoints.
                        newPath.assign(oldPath.begin(), oldPath.begin() + static_cast<std::ptrdiff_t>(anchor));
                        newPath.insert(newPath.end(), bridge.begin(), bridge.end());
                        newPath.insert(newPath.end(),
                            oldPath.begin() + static_cast<std::ptrdiff_t>(reconnectIndex + 1), oldPath.end());
                        if (!structurallyValid(newPath, oldPath.front(), oldPath.back())) {
                            continue;
                        }
                        nextConfiguration = configurationFingerprint(result.paths, activeIndex, newPath);
                        if (visitedConfigurations.contains(nextConfiguration)) {
                            continue;
                        }
                        repairStartIndex = static_cast<int>(anchor);
                        repairSuccess = true;
                        break;
                    }
                }
            }

            if (!repairSuccess && !oldPath.empty()) {
                newPath = sipp.solve(grid, oldPath.front(), oldPath.back(),
                    result.reservations.safeIntervalTable, 0, AStarSippSolver::GoalOccupation::Permanent);
                if (structurallyValid(newPath, oldPath.front(), oldPath.back())) {
                    nextConfiguration = configurationFingerprint(result.paths, activeIndex, newPath);
                    repairSuccess = !visitedConfigurations.contains(nextConfiguration);
                    repairStartIndex = 0;
                }
            }
            if (!repairSuccess) {
                // Restore before returning: result may be moved before local
                // destructors run when named return value optimization is absent.
                repair.rollback();
                updateResultMetrics(result, agents, false, startedAt);
                return result;
            }

            repair.insertPath(newPath, agents[activeIndex].id);

            SolutionConflicts updatedConflicts = UpdateSolutionConflictsV2(
                newPath, result.reservations, result.remainingConflicts,
                static_cast<int>(activeIndex), repairStartIndex);
            visitedConfigurations.insert(std::move(nextConfiguration));
            // All allocating work finished. Publishing cannot leave paths and
            // reservations inconsistent if an earlier operation throws.
            static_assert(std::is_nothrow_move_assignable_v<decltype(newPath)>);
            static_assert(std::is_nothrow_move_assignable_v<SolutionConflicts>);
            result.paths[activeIndex] = std::move(newPath);
            result.remainingConflicts = std::move(updatedConflicts);
            repair.commit();
        }

        updateResultMetrics(result, agents, true, startedAt);
        return result;
    }

}
