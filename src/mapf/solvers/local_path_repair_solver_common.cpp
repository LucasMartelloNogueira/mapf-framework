#include "local_path_repair_solver_common.hpp"

#include "mapf/pathfinding/a_star_sipp.hpp"
#include "mapf/utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <iterator>
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
            std::size_t revision;
            std::variant<CellConflict, EdgeConflict> conflict;
        };

        struct RepairWindow {
            std::size_t prefixEnd;
            std::size_t reconnectIndex;
            std::size_t endOldInclusive;
            std::optional<int> nextConflictTime;
        };

        struct LocalCandidate {
            std::vector<Cell*> path;
            int validatedThrough;
            int suffixTimeShift;
        };

        struct CandidateSnapshot {
            std::vector<std::list<Cell*>> paths;
            SolutionConflicts conflicts;
            std::string fingerprint;
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

        const Interval* intervalAt(
            const SafeIntervalTable& table,
            Cell* cell,
            int time
        ) {
            const std::unordered_map<Cell*, std::vector<Interval>>::const_iterator cellIntervals = table.safeIntervalsByCell.find(cell);
            if (cellIntervals == table.safeIntervalsByCell.end()) {
                return nullptr;
            }

            for (const Interval& interval : cellIntervals->second) {
                if (interval.start <= time && time <= interval.end) {
                    return &interval;
                }
            }

            return nullptr;
        }

        bool edgeBlocked(
            const SafeIntervalTable& table,
            Cell* from,
            Cell* to,
            int arrivalTime
        ) {
            if (from == to) {
                return false;
            }

            const std::unordered_map<EdgeKey, std::unordered_set<int>, EdgeKeyHash>::const_iterator blocked = table.blockedEdgeArrivals.find({from, to});
            return blocked != table.blockedEdgeArrivals.end() &&
                blocked->second.contains(arrivalTime);
        }

        bool hasOtherAgent(
            const VertexOccupants& vertexAgents,
            Cell* cell,
            int activeAgentId
        ) {
            const auto agentsAtCell = vertexAgents.find(cell);
            if (agentsAtCell == vertexAgents.end()) {
                return false;
            }

            for (const auto& [time, occupants] : agentsAtCell->second) {
                if (std::any_of(occupants.begin(), occupants.end(), [activeAgentId](int agentId) {
                        return agentId != activeAgentId;
                    })) {
                    return true;
                }
            }
            return false;
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

        bool structurallyValid(
            const std::vector<Cell*>& path,
            Cell* expectedStart,
            Cell* expectedGoal
        ) {
            if (path.empty() || path.front() != expectedStart || path.back() != expectedGoal) {
                return false;
            }

            for (std::size_t i = 0; i < path.size(); i++) {
                if (path[i] == nullptr || !path[i]->isFree) {
                    return false;
                }

                if (i > 0 && !isValidStep(path[i - 1], path[i])) {
                    return false;
                }
            }

            return true;
        }

        bool sameRecord(
            const std::variant<CellConflict, EdgeConflict>& first,
            const std::variant<CellConflict, EdgeConflict>& second
        ) {
            if (first.index() != second.index()) {
                return false;
            }
            const CellConflict* vertex = std::get_if<CellConflict>(&first);
            if (vertex != nullptr) {
                const CellConflict& other = std::get<CellConflict>(second);
                return vertex->cell == other.cell && vertex->time == other.time;
            }
            const EdgeConflict& edge = std::get<EdgeConflict>(first);
            const EdgeConflict& other = std::get<EdgeConflict>(second);
            return edge.cell_1 == other.cell_1 && edge.cell_2 == other.cell_2 && edge.time == other.time;
        }

        std::string unresolvedConflictFingerprint(
            std::size_t pathIndex,
            const std::variant<CellConflict, EdgeConflict>& conflict
        ) {
            const CellConflict* vertex = std::get_if<CellConflict>(&conflict);
            Cell* first = vertex != nullptr ? vertex->cell : std::get<EdgeConflict>(conflict).cell_1;
            Cell* second = vertex != nullptr ? vertex->cell : std::get<EdgeConflict>(conflict).cell_2;
            std::ostringstream output;
            output << pathIndex << '|' << conflict.index() << '|' << conflictTime(conflict)
                << '|' << first->position.x << ',' << first->position.y
                << '|' << second->position.x << ',' << second->position.y;
            return output.str();
        }

        std::optional<SelectedConflict> firstEligibleConflict(
            std::size_t pathIndex,
            const SolutionConflicts& conflicts,
            const std::unordered_set<std::string>& ignored,
            std::size_t& cursor,
            std::size_t revision
        ) {
            const std::vector<std::variant<CellConflict, EdgeConflict>>& records = conflicts.byAgent[pathIndex];
            while (cursor < records.size()) {
                if (!ignored.contains(unresolvedConflictFingerprint(pathIndex, records[cursor]))) {
                    return SelectedConflict{pathIndex, revision, records[cursor]};
                }
                ++cursor;
            }
            return std::nullopt;
        }

        std::optional<SelectedConflict> selectByAgent(
            const SolutionConflicts& conflicts,
            const std::unordered_set<std::string>& ignored,
            std::vector<std::size_t>& cursors,
            std::size_t revision
        ) {
            for (std::size_t i = 0; i < conflicts.byAgent.size(); ++i) {
                const std::optional<SelectedConflict> candidate = firstEligibleConflict(
                    i, conflicts, ignored, cursors[i], revision);
                if (candidate) {
                    return candidate;
                }
            }
            return std::nullopt;
        }

        std::optional<SelectedConflict> selectByTime(
            const SolutionConflicts& conflicts,
            const std::unordered_set<std::string>& ignored,
            std::vector<std::size_t>& cursors,
            std::size_t revision
        ) {
            std::optional<SelectedConflict> earliest;
            for (std::size_t i = 0; i < conflicts.byAgent.size(); ++i) {
                const std::optional<SelectedConflict> candidate = firstEligibleConflict(
                    i, conflicts, ignored, cursors[i], revision);
                // Iteration order breaks identical-record ties by path index.
                if (candidate && (!earliest || conflictRecordComesBefore(
                    candidate->conflict, earliest->conflict))) {
                    earliest = candidate;
                }
            }
            return earliest;
        }

        bool selectionIsCurrent(
            const SelectedConflict& selected,
            const SolutionConflicts& conflicts,
            std::size_t revision
        ) {
            if (selected.revision != revision || selected.pathIndex >= conflicts.byAgent.size()) {
                return false;
            }
            const std::vector<std::variant<CellConflict, EdgeConflict>>& records =
                conflicts.byAgent[selected.pathIndex];
            const std::vector<std::variant<CellConflict, EdgeConflict>>::const_iterator record =
                std::lower_bound(records.begin(), records.end(), selected.conflict, conflictRecordComesBefore);
            if (record == records.end() || !sameRecord(*record, selected.conflict)) {
                return false;
            }
            const int owner = static_cast<int>(selected.pathIndex);
            const CellConflict* vertex = std::get_if<CellConflict>(&selected.conflict);
            if (vertex != nullptr) {
                const std::unordered_map<CellTime, VertexEvent, CellTimeHash>::const_iterator event =
                    conflicts.vertexEvents.find({vertex->cell, vertex->time});
                return event != conflicts.vertexEvents.end() && event->second.participants.size() > 1 &&
                    event->second.participants.contains(owner);
            }
            const EdgeConflict& edge = std::get<EdgeConflict>(selected.conflict);
            const std::unordered_map<EdgeTime, EdgeEvent, EdgeTimeHash>::const_iterator event =
                conflicts.edgeEvents.find({edge.cell_1, edge.cell_2, edge.time});
            return event != conflicts.edgeEvents.end() && event->second.active() &&
                (event->second.forward.contains(owner) || event->second.reverse.contains(owner));
        }

        std::optional<RepairWindow> makeRepairWindow(
            const SelectedConflict& selected,
            std::size_t pathSize,
            const std::vector<std::variant<CellConflict, EdgeConflict>>& conflicts
        ) {
            const int time = conflictTime(selected.conflict);
            if (time <= 0 || pathSize == 0 || static_cast<std::size_t>(time) >= pathSize) {
                return std::nullopt;
            }
            std::optional<int> nextTime;
            for (const std::variant<CellConflict, EdgeConflict>& conflict : conflicts) {
                if (conflictTime(conflict) > time) {
                    nextTime = conflictTime(conflict);
                    break;
                }
            }
            const std::size_t end = nextTime ? static_cast<std::size_t>(*nextTime - 1) : pathSize - 1;
            const std::size_t reconnect = static_cast<std::size_t>(time) +
                (std::holds_alternative<CellConflict>(selected.conflict) ? 1 : 0);
            const std::size_t prefix = static_cast<std::size_t>(time - 1);
            if (prefix >= reconnect || reconnect > end || end >= pathSize) {
                return std::nullopt;
            }
            return RepairWindow{prefix, reconnect, end, nextTime};
        }

        std::string configurationFingerprint(const std::vector<std::list<Cell*>>& paths) {
            std::ostringstream output;
            output << paths.size() << ':';
            for (const std::list<Cell*>& path : paths) {
                output << '[' << path.size() << ':';
                for (Cell* cell : path) {
                    output << cell->position.x << ',' << cell->position.y << ';';
                }
                output << ']';
            }
            return output.str();
        }

        std::string attemptFingerprint(
            const std::string& configuration,
            const SelectedConflict& selected,
            const std::unordered_set<std::string>& ignored
        ) {
            std::vector<std::string> exclusions(ignored.begin(), ignored.end());
            std::sort(exclusions.begin(), exclusions.end());
            std::ostringstream output;
            output << configuration << '|' << unresolvedConflictFingerprint(selected.pathIndex, selected.conflict);
            for (const std::string& exclusion : exclusions) {
                output << '[' << exclusion << ']';
            }
            return output.str();
        }

        bool activeAgentProgress(
            const SolutionConflicts& before,
            const SolutionConflicts& after,
            const SelectedConflict& selected,
            const std::unordered_set<std::string>& ignored,
            int oldArrival,
            int newArrival,
            int changedFrom,
            int validatedThrough,
            bool fullReplacement
        ) {
            const int owner = static_cast<int>(selected.pathIndex);
            const int selectedTime = conflictTime(selected.conflict);
            // All explicit events in the changed segment, including same-time
            // vertex/swap ties, must disappear. Later deferred events may remain.
            for (const std::variant<CellConflict, EdgeConflict>& record : after.byAgent[selected.pathIndex]) {
                const int time = conflictTime(record);
                if (fullReplacement || time == selectedTime ||
                    (time >= changedFrom && time <= validatedThrough)) {
                    return false;
                }
            }
            // Global membership also covers an active agent arriving early and
            // becoming a virtual goal owner, which has no byAgent record.
            for (const std::pair<const CellTime, VertexEvent>& entry : after.vertexEvents) {
                if (!entry.second.participants.contains(owner)) {
                    continue;
                }
                const int time = entry.first.time;
                if (fullReplacement || time == selectedTime ||
                    (time >= changedFrom && time <= validatedThrough)) {
                    return false;
                }
                if (time < selectedTime) {
                    const std::unordered_map<CellTime, VertexEvent, CellTimeHash>::const_iterator previous =
                        before.vertexEvents.find(entry.first);
                    if (!ignored.contains(unresolvedConflictFingerprint(selected.pathIndex,
                            CellConflict{entry.first.cell, time})) ||
                        previous == before.vertexEvents.end() ||
                        previous->second.participants != entry.second.participants ||
                        (time <= oldArrival) != (time <= newArrival)) {
                        return false;
                    }
                }
            }
            for (const std::pair<const EdgeTime, EdgeEvent>& entry : after.edgeEvents) {
                if (!entry.second.forward.contains(owner) && !entry.second.reverse.contains(owner)) {
                    continue;
                }
                const int time = entry.first.time;
                if (fullReplacement || time == selectedTime ||
                    (time >= changedFrom && time <= validatedThrough)) {
                    return false;
                }
                if (time < selectedTime) {
                    const std::unordered_map<EdgeTime, EdgeEvent, EdgeTimeHash>::const_iterator previous =
                        before.edgeEvents.find(entry.first);
                    if (!ignored.contains(unresolvedConflictFingerprint(selected.pathIndex,
                            EdgeConflict{entry.first.first, entry.first.second, time})) ||
                        previous == before.edgeEvents.end() ||
                        previous->second.forward != entry.second.forward ||
                        previous->second.reverse != entry.second.reverse) {
                        return false;
                    }
                }
            }
            return true;
        }

        std::optional<CandidateSnapshot> evaluateCandidate(
            const LocalPathRepairResult& result,
            const SelectedConflict& selected,
            const std::vector<Cell*>& candidate,
            const std::unordered_set<std::string>& ignored,
            const std::unordered_set<std::string>& committedConfigurations,
            int changedFrom,
            int validatedThrough,
            bool fullReplacement
        ) {
            CandidateSnapshot provisional;
            provisional.paths = result.paths;
            provisional.paths[selected.pathIndex] = std::list<Cell*>(candidate.begin(), candidate.end());
            provisional.conflicts = getCollision(provisional.paths);
            if (!activeAgentProgress(result.remainingConflicts, provisional.conflicts, selected, ignored,
                pathCost(result.paths[selected.pathIndex]), static_cast<int>(candidate.size() - 1),
                changedFrom, validatedThrough, fullReplacement)) {
                return std::nullopt;
            }
            provisional.fingerprint = configurationFingerprint(provisional.paths);
            if (committedConfigurations.contains(provisional.fingerprint)) {
                return std::nullopt;
            }
            return provisional;
        }

        std::optional<int> earliestSafeArrival(
            const SafeIntervalTable& table,
            Cell* current,
            Cell* next,
            int currentTime,
            bool requirePermanentGoal
        ) {
            if (currentTime < 0 || currentTime >= SAFE_INTERVAL_INFINITY - 1) {
                return std::nullopt;
            }
            const Interval* currentInterval = intervalAt(table, current, currentTime);
            if (currentInterval == nullptr) {
                return std::nullopt;
            }

            const std::unordered_map<Cell*, std::vector<Interval>>::const_iterator nextIntervals = table.safeIntervalsByCell.find(next);
            if (nextIntervals == table.safeIntervalsByCell.end()) {
                return std::nullopt;
            }

            const int latestArrival = currentInterval->end >= SAFE_INTERVAL_INFINITY
                ? SAFE_INTERVAL_INFINITY
                : currentInterval->end + 1;

            for (const Interval& nextInterval : nextIntervals->second) {
                if (requirePermanentGoal && nextInterval.end < SAFE_INTERVAL_INFINITY) {
                    continue;
                }

                int arrival = std::max(currentTime + 1, nextInterval.start);
                const int intervalLatest = std::min({latestArrival, nextInterval.end, SAFE_INTERVAL_INFINITY - 1});

                while (arrival <= intervalLatest && edgeBlocked(table, current, next, arrival)) {
                    arrival++;
                }

                if (arrival <= intervalLatest) {
                    return arrival;
                }
            }

            return std::nullopt;
        }

        void appendWithoutFirst(
            std::vector<Cell*>& destination,
            const std::list<Cell*>& segment
        ) {
            bool first = true;
            for (Cell* cell : segment) {
                if (first) {
                    first = false;
                    continue;
                }

                destination.push_back(cell);
            }
        }

        bool appendDirectTransition(
            std::vector<Cell*>& candidate,
            Cell* next,
            const SafeIntervalTable& table,
            bool requirePermanentGoal
        ) {
            if (candidate.empty() || candidate.size() >= static_cast<std::size_t>(SAFE_INTERVAL_INFINITY)) {
                return false;
            }
            Cell* current = candidate.back();
            if (next == nullptr || !next->isFree || !isValidStep(current, next)) {
                return false;
            }

            const int arrivalTime = static_cast<int>(candidate.size());
            const Interval* nextInterval = intervalAt(table, next, arrivalTime);
            if (
                nextInterval == nullptr ||
                (requirePermanentGoal && nextInterval->end < SAFE_INTERVAL_INFINITY) ||
                edgeBlocked(table, current, next, arrivalTime)
            ) {
                return false;
            }

            candidate.push_back(next);
            return true;
        }

        bool appendValidatedSegment(
            std::vector<Cell*>& candidate,
            const std::list<Cell*>& segment,
            const SafeIntervalTable& table
        ) {
            if (segment.empty() || candidate.empty() || segment.front() != candidate.back() ||
                candidate.size() > static_cast<std::size_t>(SAFE_INTERVAL_INFINITY) ||
                segment.size() - 1 > static_cast<std::size_t>(SAFE_INTERVAL_INFINITY) - candidate.size() ||
                intervalAt(table, candidate.back(), static_cast<int>(candidate.size() - 1)) == nullptr) {
                return false;
            }
            // Validate only the generated segment before splicing it.
            Cell* previous = candidate.back();
            std::size_t arrival = candidate.size();
            for (std::list<Cell*>::const_iterator cell = std::next(segment.begin()); cell != segment.end(); ++cell) {
                if (*cell == nullptr || !(*cell)->isFree || !isValidStep(previous, *cell) ||
                    intervalAt(table, *cell, static_cast<int>(arrival)) == nullptr ||
                    edgeBlocked(table, previous, *cell, static_cast<int>(arrival))) {
                    return false;
                }
                previous = *cell;
                ++arrival;
            }
            appendWithoutFirst(candidate, segment);
            return true;
        }

        bool appendScenarioTwoPointThreeSuffix(
            Grid& grid,
            const Agent& agent,
            const std::vector<Cell*>& oldPath,
            std::size_t suffixStartIndex,
            std::size_t endOldInclusive,
            const PathReservationState& committedState,
            const PathReservationState& repairState,
            std::vector<Cell*>& candidate
        ) {
            if (suffixStartIndex > endOldInclusive || endOldInclusive >= oldPath.size()) {
                return false;
            }
            AStarSippSolver sipp;

            for (
                std::size_t oldIndex = suffixStartIndex;
                oldIndex < endOldInclusive;
                oldIndex++
            ) {
                Cell* current = candidate.back();
                Cell* next = oldPath[oldIndex + 1];
                const bool nextIsGoal = oldIndex + 1 == oldPath.size() - 1;

                if (current == next) {
                    if (!appendDirectTransition(
                        candidate,
                        next,
                        repairState.safeIntervalTable,
                        nextIsGoal
                    )) {
                        return false;
                    }

                    continue;
                }

                const bool currentHasPotentialConflict = hasOtherAgent(
                    committedState.vertex_agents,
                    current,
                    agent.id
                );
                const bool nextHasPotentialConflict = hasOtherAgent(
                    committedState.vertex_agents,
                    next,
                    agent.id
                );

                // cenário 3
                if (!currentHasPotentialConflict && nextHasPotentialConflict) {
                    const int currentTime = static_cast<int>(candidate.size()) - 1;
                    const std::optional<int> arrivalTime = earliestSafeArrival(
                        repairState.safeIntervalTable,
                        current,
                        next,
                        currentTime,
                        nextIsGoal
                    );
                    if (!arrivalTime) {
                        return false;
                    }

                    for (int time = currentTime + 1; time < *arrivalTime; time++) {
                        candidate.push_back(current);
                    }

                    candidate.push_back(next);
                    continue;
                }

                // cenário 4
                if (currentHasPotentialConflict && nextHasPotentialConflict) {
                    const int currentTime = static_cast<int>(candidate.size()) - 1;
                    std::list<Cell*> miniPath = sipp.solve(
                        grid,
                        current,
                        next,
                        repairState.safeIntervalTable,
                        currentTime
                    );
                    if (miniPath.empty()) {
                        return false;
                    }

                    if (!appendValidatedSegment(candidate, miniPath, repairState.safeIntervalTable)) {
                        return false;
                    }
                    if (nextIsGoal) {
                        const Interval* goalInterval = intervalAt(
                            repairState.safeIntervalTable,
                            next,
                            static_cast<int>(candidate.size()) - 1
                        );
                        if (
                            goalInterval == nullptr ||
                            goalInterval->end < SAFE_INTERVAL_INFINITY
                        ) {
                            return false;
                        }
                    }

                    continue;
                }

                if (!appendDirectTransition(
                    candidate,
                    next,
                    repairState.safeIntervalTable,
                    nextIsGoal
                )) {
                    return false;
                }
            }

            return true;
        }

        std::optional<LocalCandidate> buildLocalCandidate(
            Grid& grid,
            const Agent& agent,
            const std::vector<Cell*>& oldPath,
            const RepairWindow& window,
            const std::list<Cell*>& bridge,
            const PathReservationState& committedState,
            const PathReservationState& repairState
        ) {
            if (bridge.empty() || window.prefixEnd >= window.reconnectIndex ||
                window.reconnectIndex > window.endOldInclusive || window.endOldInclusive >= oldPath.size() ||
                bridge.front() != oldPath[window.prefixEnd] || bridge.back() != oldPath[window.reconnectIndex]) {
                return std::nullopt;
            }
            std::vector<Cell*> candidate(oldPath.begin(),
                oldPath.begin() + static_cast<std::ptrdiff_t>(window.prefixEnd + 1));
            if (!appendValidatedSegment(candidate, bridge, repairState.safeIntervalTable)) {
                return std::nullopt;
            }

            const int originalArrival = static_cast<int>(window.reconnectIndex);
            const int newArrival = static_cast<int>(candidate.size() - 1);
            bool potentialConflict = false;
            for (std::size_t i = window.reconnectIndex; i <= window.endOldInclusive; ++i) {
                if (hasOtherAgent(committedState.vertex_agents, oldPath[i], agent.id)) {
                    potentialConflict = true;
                    break;
                }
            }

            // Earlier arrival with interference: wait in one safe interval.
            if (newArrival < originalArrival && potentialConflict) {
                const Interval* interval = intervalAt(repairState.safeIntervalTable,
                    candidate.back(), newArrival);
                if (interval == nullptr || interval->end < originalArrival) {
                    return std::nullopt;
                }
                candidate.insert(candidate.end(), static_cast<std::size_t>(originalArrival - newArrival),
                    oldPath[window.reconnectIndex]);
            }

            if (newArrival > originalArrival && potentialConflict) {
                if (!appendScenarioTwoPointThreeSuffix(grid, agent, oldPath, window.reconnectIndex,
                    window.endOldInclusive, committedState, repairState, candidate)) {
                    return std::nullopt;
                }
            } else {
                // Equal/earlier/uncontested-later cases still validate each
                // bounded transition; a spatial lookup alone is not safety.
                for (std::size_t i = window.reconnectIndex; i < window.endOldInclusive; ++i) {
                    if (!appendDirectTransition(candidate, oldPath[i + 1], repairState.safeIntervalTable,
                        i + 1 == oldPath.size() - 1)) {
                        return std::nullopt;
                    }
                }
            }

            const int validatedThrough = static_cast<int>(candidate.size() - 1);
            const int shift = validatedThrough - static_cast<int>(window.endOldInclusive);
            const std::size_t tailSize = oldPath.size() - window.endOldInclusive - 1;
            if (tailSize > static_cast<std::size_t>(SAFE_INTERVAL_INFINITY) - candidate.size()) {
                return std::nullopt;
            }
            if (tailSize > 0 && !isValidStep(candidate.back(), oldPath[window.endOldInclusive + 1])) {
                return std::nullopt;
            }
            // Only copying beyond the bound: temporal evaluation belongs to
            // the provisional global snapshot, including the first tail edge.
            candidate.insert(candidate.end(),
                oldPath.begin() + static_cast<std::ptrdiff_t>(window.endOldInclusive + 1), oldPath.end());
            const Interval* goalInterval = intervalAt(repairState.safeIntervalTable,
                candidate.back(), static_cast<int>(candidate.size() - 1));
            if (goalInterval == nullptr || goalInterval->end < SAFE_INTERVAL_INFINITY) {
                return std::nullopt;
            }
            return LocalCandidate{std::move(candidate), validatedThrough, shift};
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
        // Validate every membership before changing any entry. All later allocation
        // failures affect only this disposable state, never the committed result.
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
        bool continueIfFailed,
        std::chrono::steady_clock::time_point startedAt,
        LocalRepairStrategy localRepairStrategy
    ) {
        validateLocalRepairStrategy(localRepairStrategy); // TODO: não validar aqui, essa validação será feita na validação dos args da CLI
        const std::vector<Agent>& agents = instance.getAgents();
        if (initialPaths.size() != agents.size()) {
            throw std::invalid_argument("Initial paths must be aligned with instance agents.");
        }
        Grid& grid = const_cast<Grid&>(instance.getGrid());
        bool allInitialPathsExist = true;
        for (std::size_t i = 0; i < initialPaths.size(); ++i) {
            if (initialPaths[i].empty()) {
                allInitialPathsExist = false;
                continue;
            }
            const std::vector<Cell*> path(initialPaths[i].begin(), initialPaths[i].end());
            if (path.size() > static_cast<std::size_t>(SAFE_INTERVAL_INFINITY) ||
                !structurallyValid(path,
                    grid.getCellPtr(agents[i].startPosition.x, agents[i].startPosition.y),
                    grid.getCellPtr(agents[i].goalPosition.x, agents[i].goalPosition.y))) {
                throw std::invalid_argument("Malformed initial path or unsupported SIPP time.");
            }
        }

        LocalPathRepairResult result;
        result.initialPaths = std::move(initialPaths);
        result.paths = result.initialPaths;
        result.initialConflicts = getCollision(result.initialPaths);
        result.remainingConflicts = result.initialConflicts;
        result.reservations = buildReservationState(grid, agents, result.paths);
        if (!allInitialPathsExist || agents.empty()) {
            updateResultMetrics(result, agents, allInitialPathsExist, startedAt);
            return result;
        }

        AStarSippSolver sipp;
        std::size_t revision = 0;
        std::vector<std::size_t> cursors(agents.size(), 0);
        std::unordered_set<std::string> ignoredForRevision;
        std::unordered_set<std::string> committedConfigurations;
        std::unordered_set<std::string> attemptedSelections;
        std::string configuration = configurationFingerprint(result.paths);
        committedConfigurations.insert(configuration);

        while (true) {
            std::optional<SelectedConflict> selected;
            switch (localRepairStrategy) {
            case LocalRepairStrategy::RESOLVE_BY_AGENT:
                selected = selectByAgent(result.remainingConflicts, ignoredForRevision, cursors, revision);
                break;
            case LocalRepairStrategy::RESOLVE_BY_TIME:
                selected = selectByTime(result.remainingConflicts, ignoredForRevision, cursors, revision);
                break;
            default:
                throw std::invalid_argument("Unknown local repair strategy.");
            }
            if (!selected) {
                break;
            }
            if (!selectionIsCurrent(*selected, result.remainingConflicts, revision)) {
                throw std::logic_error("Stale local repair selection.");
            }
            const std::size_t activeIndex = selected->pathIndex;
            const int selectedTime = conflictTime(selected->conflict);
            const bool firstAttempt = attemptedSelections.insert(
                attemptFingerprint(configuration, *selected, ignoredForRevision)).second;
            std::optional<CandidateSnapshot> accepted;
            std::optional<PathReservationState> preparedReservations;

            if (firstAttempt) {
                const std::vector<Cell*> oldPath(result.paths[activeIndex].begin(), result.paths[activeIndex].end());
                const std::optional<RepairWindow> initialWindow = makeRepairWindow(
                    *selected, oldPath.size(), result.remainingConflicts.byAgent[activeIndex]);
                preparedReservations.emplace(result.reservations);
                PathReservationState& repairState = *preparedReservations;
                repairSafeIntervalTable(repairState, result.paths[activeIndex], agents[activeIndex].id);
                if (initialWindow) {
                    for (std::size_t anchor : adaptiveAnchors(initialWindow->prefixEnd)) {
                        RepairWindow window = *initialWindow;
                        window.prefixEnd = anchor;
                        const std::list<Cell*> bridge = sipp.solve(grid, oldPath[anchor],
                            oldPath[window.reconnectIndex], repairState.safeIntervalTable, static_cast<int>(anchor));
                        const std::optional<LocalCandidate> candidate = buildLocalCandidate(
                            grid, agents[activeIndex], oldPath, window, bridge, result.reservations, repairState);
                        if (!candidate || (window.nextConflictTime && candidate->validatedThrough < selectedTime)) {
                            continue;
                        }
                        accepted = evaluateCandidate(result, *selected, candidate->path, ignoredForRevision,
                            committedConfigurations, static_cast<int>(anchor), candidate->validatedThrough, false);
                        if (accepted) {
                            break;
                        }
                    }
                }
                if (!accepted) {
                    const std::list<Cell*> fullPath = sipp.solve(grid, oldPath.front(), oldPath.back(),
                        repairState.safeIntervalTable, 0, AStarSippSolver::GoalOccupation::Permanent);
                    const std::vector<Cell*> fullCandidate(fullPath.begin(), fullPath.end());
                    if (fullCandidate.size() <= static_cast<std::size_t>(SAFE_INTERVAL_INFINITY) &&
                        structurallyValid(fullCandidate, oldPath.front(), oldPath.back())) {
                        accepted = evaluateCandidate(result, *selected, fullCandidate, ignoredForRevision,
                            committedConfigurations, 0, static_cast<int>(fullCandidate.size() - 1), true);
                    }
                }
            }

            if (!accepted) {
                if (!continueIfFailed) {
                    updateResultMetrics(result, agents, false, startedAt);
                    return result;
                }
                ignoredForRevision.insert(unresolvedConflictFingerprint(activeIndex, selected->conflict));
                continue;
            }

            // Finish all allocating work before publishing the new revision.
            updateReservationState(*preparedReservations, accepted->paths[activeIndex], agents[activeIndex].id);
            if (revision == std::numeric_limits<std::size_t>::max()) {
                throw std::overflow_error("Local repair revision overflow.");
            }
            committedConfigurations.insert(accepted->fingerprint);
            static_assert(std::is_nothrow_move_assignable_v<decltype(configuration)>);
            static_assert(std::is_nothrow_move_assignable_v<decltype(result.paths)>);
            static_assert(std::is_nothrow_move_assignable_v<SolutionConflicts>);
            static_assert(std::is_nothrow_move_assignable_v<PathReservationState>);
            configuration = std::move(accepted->fingerprint);
            result.paths = std::move(accepted->paths);
            result.remainingConflicts = std::move(accepted->conflicts);
            result.reservations = std::move(*preparedReservations);
            ++revision;
            ignoredForRevision.clear();
            std::fill(cursors.begin(), cursors.end(), 0);
        }

        updateResultMetrics(result, agents, true, startedAt);
        return result;
    }

}
