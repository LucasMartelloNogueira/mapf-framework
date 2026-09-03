#include "mapf/solvers/local_path_repair_parallel_solver.hpp"

#include "mapf/pathfinding/a_star.hpp"
#include "mapf/pathfinding/a_star_sipp.hpp"
#include "mapf/utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mapf {

    namespace {
        class ThreadPool {
            private:
                std::vector<std::jthread> workers;
                std::queue<std::function<void()>> tasks;
                std::mutex tasksMutex;
                std::condition_variable ready;
                bool stopping = false;

            public:
                explicit ThreadPool(std::size_t workerCount) {
                    workers.reserve(workerCount);

                    for (std::size_t i = 0; i < workerCount; i++) {
                        workers.emplace_back([this]() {
                            while (true) {
                                std::function<void()> task;

                                {
                                    std::unique_lock lock(tasksMutex);
                                    ready.wait(lock, [this]() {
                                        return stopping || !tasks.empty();
                                    });

                                    if (stopping && tasks.empty()) {
                                        return;
                                    }

                                    task = std::move(tasks.front());
                                    tasks.pop();
                                }

                                task();
                            }
                        });
                    }
                }

                ThreadPool(const ThreadPool&) = delete;
                ThreadPool& operator=(const ThreadPool&) = delete;

                ~ThreadPool() {
                    {
                        std::lock_guard lock(tasksMutex);
                        stopping = true;
                    }

                    ready.notify_all();
                    workers.clear();
                }

                template<class Function>
                auto submit(Function&& function)
                    -> std::future<std::invoke_result_t<std::decay_t<Function>&>>
                {
                    using ReturnType = std::invoke_result_t<std::decay_t<Function>&>;
                    auto task = std::make_shared<std::packaged_task<ReturnType()>>(
                        std::forward<Function>(function)
                    );
                    std::future<ReturnType> future = task->get_future();

                    {
                        std::lock_guard lock(tasksMutex);
                        if (stopping) {
                            throw std::runtime_error("Cannot submit work to a stopped thread pool.");
                        }

                        tasks.emplace([task]() {
                            (*task)();
                        });
                    }

                    ready.notify_one();
                    return future;
                }
        };

        enum class ConflictKind {
            Vertex,
            Edge
        };

        struct ActiveConflict {
            ConflictKind kind;
            int time;
            Cell* first;
            Cell* second;
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

                if (current.start <= previous.end + 1) {
                    previous.end = std::max(previous.end, current.end);
                } else {
                    merged.push_back(current);
                }
            }

            intervals = std::move(merged);
        }

        PathReservationState buildReservationState(
            Grid& grid,
            const std::vector<Agent>& agents,
            const std::vector<std::list<Cell*>>& paths,
            std::optional<std::size_t> excludedPathIndex = std::nullopt
        ) {
            PathReservationState state;
            std::unordered_map<Cell*, std::vector<Interval>> blockedIntervalsByCell;

            const std::size_t includedCount = std::min(agents.size(), paths.size());
            for (std::size_t pathIndex = 0; pathIndex < includedCount; pathIndex++) {
                if (excludedPathIndex && *excludedPathIndex == pathIndex) {
                    continue;
                }

                const std::list<Cell*>& pathList = paths[pathIndex];
                if (pathList.empty()) {
                    continue;
                }

                std::vector<Cell*> path(pathList.begin(), pathList.end());
                for (int time = 0; time < static_cast<int>(path.size()); time++) {
                    Cell* cell = path[time];
                    state.vertex_agents[cell].insert(agents[pathIndex].id);
                    blockedIntervalsByCell[cell].push_back({time, time});

                    if (time > 0 && path[time - 1] != cell) {
                        state.safeIntervalTable
                            .blockedEdgeArrivals[{cell, path[time - 1]}]
                            .insert(time);
                    }
                }

                Cell* goal = path.back();
                const int arrivalTime = static_cast<int>(path.size()) - 1;
                blockedIntervalsByCell[goal].push_back({
                    arrivalTime,
                    SAFE_INTERVAL_INFINITY
                });
                state.goal_reservations.emplace(goal, arrivalTime);
            }

            state.safeIntervalTable.safeIntervalsByCell.reserve(grid.getCells().size());
            for (Cell& cell : grid.getCells()) {
                Cell* cellPointer = &cell;
                std::vector<Interval> blockedIntervals = blockedIntervalsByCell[cellPointer];
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

                state.safeIntervalTable.safeIntervalsByCell.emplace(
                    cellPointer,
                    std::move(safeIntervals)
                );
            }

            return state;
        }

        const Interval* intervalAt(
            const SafeIntervalTable& table,
            Cell* cell,
            int time
        ) {
            const auto cellIntervals = table.safeIntervalsByCell.find(cell);
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

            const auto blocked = table.blockedEdgeArrivals.find({from, to});
            return blocked != table.blockedEdgeArrivals.end() &&
                blocked->second.contains(arrivalTime);
        }

        bool hasOtherAgent(
            const VertexAgents& vertexAgents,
            Cell* cell,
            int activeAgentId
        ) {
            const auto agentsAtCell = vertexAgents.find(cell);
            if (agentsAtCell == vertexAgents.end()) {
                return false;
            }

            return std::any_of(
                agentsAtCell->second.begin(),
                agentsAtCell->second.end(),
                [activeAgentId](int agentId) {
                    return agentId != activeAgentId;
                }
            );
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

        bool conflictComesBefore(
            const ActiveConflict& candidate,
            const ActiveConflict& current
        ) {
            if (candidate.time != current.time) {
                return candidate.time < current.time;
            }

            if (candidate.kind != current.kind) {
                return candidate.kind == ConflictKind::Vertex;
            }

            if (candidate.first->position.x != current.first->position.x) {
                return candidate.first->position.x < current.first->position.x;
            }

            if (candidate.first->position.y != current.first->position.y) {
                return candidate.first->position.y < current.first->position.y;
            }

            if (candidate.second == nullptr || current.second == nullptr) {
                return candidate.second == nullptr && current.second != nullptr;
            }

            if (candidate.second->position.x != current.second->position.x) {
                return candidate.second->position.x < current.second->position.x;
            }

            return candidate.second->position.y < current.second->position.y;
        }

        std::optional<ActiveConflict> earliestExplicitConflict(
            std::size_t activePathIndex,
            const std::vector<std::list<Cell*>>& paths,
            const SolutionConflicts& conflicts
        ) {
            if (activePathIndex >= paths.size() || paths[activePathIndex].empty()) {
                return std::nullopt;
            }

            const std::vector<Cell*> activePath(
                paths[activePathIndex].begin(),
                paths[activePathIndex].end()
            );
            std::optional<ActiveConflict> earliest;

            for (const CellConflict& conflict : conflicts.cellConflicts) {
                if (
                    conflict.time < 0 ||
                    conflict.time >= static_cast<int>(activePath.size()) ||
                    activePath[conflict.time] != &conflict.cell
                ) {
                    continue;
                }

                ActiveConflict candidate {
                    .kind = ConflictKind::Vertex,
                    .time = conflict.time,
                    .first = const_cast<Cell*>(&conflict.cell),
                    .second = nullptr
                };

                if (!earliest || conflictComesBefore(candidate, *earliest)) {
                    earliest = candidate;
                }
            }

            for (const EdgeConflict& conflict : conflicts.edgeConflicts) {
                if (
                    conflict.time <= 0 ||
                    conflict.time >= static_cast<int>(activePath.size())
                ) {
                    continue;
                }

                Cell* previous = activePath[conflict.time - 1];
                Cell* current = activePath[conflict.time];
                const bool matchesForward =
                    previous == &conflict.cell_1 && current == &conflict.cell_2;
                const bool matchesReverse =
                    previous == &conflict.cell_2 && current == &conflict.cell_1;

                if (!matchesForward && !matchesReverse) {
                    continue;
                }

                ActiveConflict candidate {
                    .kind = ConflictKind::Edge,
                    .time = conflict.time,
                    .first = previous,
                    .second = current
                };

                if (!earliest || conflictComesBefore(candidate, *earliest)) {
                    earliest = candidate;
                }
            }

            return earliest;
        }

        Cell* positionAt(const std::vector<Cell*>& path, int time) {
            if (time < static_cast<int>(path.size())) {
                return path[time];
            }

            return path.back();
        }

        std::optional<int> earliestConflictWithFrozenPaths(
            const std::vector<Cell*>& candidate,
            std::size_t activePathIndex,
            const std::vector<std::list<Cell*>>& paths
        ) {
            if (candidate.empty()) {
                return 0;
            }

            std::optional<int> earliest;

            for (std::size_t otherIndex = 0; otherIndex < paths.size(); otherIndex++) {
                if (otherIndex == activePathIndex || paths[otherIndex].empty()) {
                    continue;
                }

                const std::vector<Cell*> otherPath(
                    paths[otherIndex].begin(),
                    paths[otherIndex].end()
                );
                const int makespan = std::max(
                    static_cast<int>(candidate.size()),
                    static_cast<int>(otherPath.size())
                ) - 1;

                for (int time = 0; time <= makespan; time++) {
                    if (positionAt(candidate, time) == positionAt(otherPath, time)) {
                        if (!earliest || time < *earliest) {
                            earliest = time;
                        }
                        break;
                    }

                    if (time == 0) {
                        continue;
                    }

                    Cell* candidatePrevious = positionAt(candidate, time - 1);
                    Cell* candidateCurrent = positionAt(candidate, time);
                    Cell* otherPrevious = positionAt(otherPath, time - 1);
                    Cell* otherCurrent = positionAt(otherPath, time);

                    if (
                        candidatePrevious != candidateCurrent &&
                        otherPrevious != otherCurrent &&
                        candidatePrevious == otherCurrent &&
                        candidateCurrent == otherPrevious
                    ) {
                        if (!earliest || time < *earliest) {
                            earliest = time;
                        }
                        break;
                    }
                }
            }

            return earliest;
        }

        std::string repairFingerprint(
            const std::vector<Cell*>& path,
            std::optional<int> earliestConflictTime
        ) {
            std::ostringstream output;
            for (Cell* cell : path) {
                output << cell->position.x << ',' << cell->position.y << ';';
            }

            output << '|';
            if (earliestConflictTime) {
                output << *earliestConflictTime;
            } else {
                output << "none";
            }

            return output.str();
        }

        std::optional<int> earliestSafeArrival(
            const SafeIntervalTable& table,
            Cell* current,
            Cell* next,
            int currentTime,
            bool requirePermanentGoal
        ) {
            const Interval* currentInterval = intervalAt(table, current, currentTime);
            if (currentInterval == nullptr) {
                return std::nullopt;
            }

            const auto nextIntervals = table.safeIntervalsByCell.find(next);
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
                const int intervalLatest = std::min(latestArrival, nextInterval.end);

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
            Cell* current = candidate.back();
            if (!isValidStep(current, next)) {
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

        bool appendScenarioTwoPointThreeSuffix(
            Grid& grid,
            const Agent& agent,
            const std::vector<Cell*>& oldPath,
            std::size_t suffixStartIndex,
            const PathReservationState& committedState,
            const PathReservationState& repairState,
            std::vector<Cell*>& candidate
        ) {
            AStarSippSolver sipp;

            for (
                std::size_t oldIndex = suffixStartIndex;
                oldIndex + 1 < oldPath.size();
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

                    appendWithoutFirst(candidate, miniPath);
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

        std::optional<std::vector<Cell*>> buildLocalCandidate(
            Grid& grid,
            const Agent& agent,
            const std::vector<Cell*>& oldPath,
            std::size_t prefixEndIndex,
            std::size_t suffixStartIndex,
            const std::list<Cell*>& bridge,
            const PathReservationState& committedState,
            const PathReservationState& repairState
        ) {
            if (
                bridge.empty() ||
                prefixEndIndex >= oldPath.size() ||
                suffixStartIndex >= oldPath.size() ||
                bridge.front() != oldPath[prefixEndIndex] ||
                bridge.back() != oldPath[suffixStartIndex]
            ) {
                return std::nullopt;
            }

            std::vector<Cell*> candidate(
                oldPath.begin(),
                oldPath.begin() + static_cast<std::ptrdiff_t>(prefixEndIndex + 1)
            );
            appendWithoutFirst(candidate, bridge);

            const int originalSuffixArrivalTime = static_cast<int>(suffixStartIndex);
            const int newSuffixArrivalTime = static_cast<int>(candidate.size()) - 1;
            bool hasPotentialSuffixConflict = false;

            for (std::size_t i = suffixStartIndex; i < oldPath.size(); i++) {
                if (hasOtherAgent(committedState.vertex_agents, oldPath[i], agent.id)) {
                    hasPotentialSuffixConflict = true;
                    break;
                }
            }

            if (newSuffixArrivalTime == originalSuffixArrivalTime) {
                candidate.insert(
                    candidate.end(),
                    oldPath.begin() + static_cast<std::ptrdiff_t>(suffixStartIndex + 1),
                    oldPath.end()
                );
            } else if (newSuffixArrivalTime < originalSuffixArrivalTime) {
                if (hasPotentialSuffixConflict) {
                    const Interval* suffixInterval = intervalAt(
                        repairState.safeIntervalTable,
                        oldPath[suffixStartIndex],
                        newSuffixArrivalTime
                    );
                    if (
                        suffixInterval == nullptr ||
                        suffixInterval->end < originalSuffixArrivalTime
                    ) {
                        return std::nullopt;
                    }

                    const int waits =
                        originalSuffixArrivalTime - newSuffixArrivalTime;
                    for (int wait = 0; wait < waits; wait++) {
                        candidate.push_back(oldPath[suffixStartIndex]);
                    }
                }

                candidate.insert(
                    candidate.end(),
                    oldPath.begin() + static_cast<std::ptrdiff_t>(suffixStartIndex + 1),
                    oldPath.end()
                );
            } else if (hasPotentialSuffixConflict) {
                if (!appendScenarioTwoPointThreeSuffix(
                    grid,
                    agent,
                    oldPath,
                    suffixStartIndex,
                    committedState,
                    repairState,
                    candidate
                )) {
                    return std::nullopt;
                }
            } else {
                candidate.insert(
                    candidate.end(),
                    oldPath.begin() + static_cast<std::ptrdiff_t>(suffixStartIndex + 1),
                    oldPath.end()
                );
            }

            const int goalArrivalTime = static_cast<int>(candidate.size()) - 1;
            const Interval* goalInterval = intervalAt(
                repairState.safeIntervalTable,
                candidate.back(),
                goalArrivalTime
            );
            if (goalInterval == nullptr || goalInterval->end < SAFE_INTERVAL_INFINITY) {
                return std::nullopt;
            }

            return candidate;
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

        bool hasDuplicateGoals(Grid& grid, const std::vector<Agent>& agents) {
            std::unordered_set<Cell*> goals;
            for (const Agent& agent : agents) {
                Cell* goal = grid.getCellPtr(
                    agent.goalPosition.x,
                    agent.goalPosition.y
                );
                if (!goals.insert(goal).second) {
                    return true;
                }
            }

            return false;
        }

        bool hasSharedStarts(const std::vector<std::list<Cell*>>& paths) {
            std::unordered_set<Cell*> starts;
            for (const std::list<Cell*>& path : paths) {
                if (!path.empty() && !starts.insert(path.front()).second) {
                    return true;
                }
            }

            return false;
        }

        std::vector<std::list<Cell*>> pathsExcept(
            const std::vector<std::list<Cell*>>& paths,
            std::size_t excludedPathIndex
        ) {
            std::vector<std::list<Cell*>> otherPaths;
            if (!paths.empty()) {
                otherPaths.reserve(paths.size() - 1);
            }

            for (std::size_t i = 0; i < paths.size(); i++) {
                if (i != excludedPathIndex) {
                    otherPaths.push_back(paths[i]);
                }
            }

            return otherPaths;
        }

        double calculateInjustice(
            const std::vector<int>& initialCosts,
            const std::vector<int>& finalCosts,
            bool allInitialPathsExist
        ) {
            if (!allInitialPathsExist || initialCosts.empty()) {
                return 0.0;
            }

            std::vector<double> differences;
            differences.reserve(initialCosts.size());
            double sum = 0.0;

            for (std::size_t i = 0; i < initialCosts.size(); i++) {
                const double difference = finalCosts[i] - initialCosts[i];
                differences.push_back(difference);
                sum += difference;
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

        void synchronizeResult(
            LocalPathRepairResult& result,
            Grid& grid,
            const std::vector<Agent>& agents,
            const std::vector<int>& initialCosts,
            bool allInitialPathsExist,
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

            result.remainingConflicts = getCollision(result.paths);
            result.reservations = buildReservationState(grid, agents, result.paths);
            const bool conflictFree =
                result.remainingConflicts.cellConflicts.empty() &&
                result.remainingConflicts.edgeConflicts.empty();
            const bool success =
                requestedSuccess && allPathsExist && conflictFree;

            result.metrics = Result(
                success,
                sumOfCosts,
                makespan,
                calculateInjustice(
                    initialCosts,
                    result.pathCosts,
                    allInitialPathsExist
                ),
                elapsedSeconds(startedAt)
            );
        }
    }

    LocalPathRepairParallelSolver::LocalPathRepairParallelSolver(
        const Instance& instance,
        std::size_t numberOfThreads
    ) :
        instance(instance),
        numberOfThreads(numberOfThreads)
    {
        if (numberOfThreads == 0) {
            throw std::invalid_argument("The number of threads must be positive.");
        }
    }

    LocalPathRepairResult LocalPathRepairParallelSolver::solve() {
        const auto startedAt = std::chrono::steady_clock::now();
        LocalPathRepairResult result;
        const std::vector<Agent>& agents = instance.getAgents();

        if (agents.empty()) {
            result.metrics = Result(true, 0, 0, 0.0, elapsedSeconds(startedAt));
            return result;
        }

        Grid& grid = const_cast<Grid&>(instance.getGrid());
        result.paths.resize(agents.size());

        {
            const std::size_t workerCount = std::min(numberOfThreads, agents.size());
            ThreadPool pool(workerCount);
            std::vector<std::future<std::list<Cell*>>> futures;
            futures.reserve(agents.size());

            for (const Agent& agent : agents) {
                futures.push_back(pool.submit([&grid, agent]() {
                    AStarSolver solver;
                    Cell* start = grid.getCellPtr(
                        agent.startPosition.x,
                        agent.startPosition.y
                    );
                    Cell* goal = grid.getCellPtr(
                        agent.goalPosition.x,
                        agent.goalPosition.y
                    );
                    return solver.solve(grid, start, goal);
                }));
            }

            for (std::size_t i = 0; i < futures.size(); i++) {
                result.paths[i] = futures[i].get();
            }
        }

        std::vector<int> initialCosts(result.paths.size(), 0);
        bool allInitialPathsExist = true;
        for (std::size_t i = 0; i < result.paths.size(); i++) {
            if (result.paths[i].empty()) {
                allInitialPathsExist = false;
            }
            initialCosts[i] = pathCost(result.paths[i]);
        }

        result.initialConflicts = getCollision(result.paths);
        synchronizeResult(
            result,
            grid,
            agents,
            initialCosts,
            allInitialPathsExist,
            false,
            startedAt
        );

        if (!allInitialPathsExist || hasSharedStarts(result.paths) || hasDuplicateGoals(grid, agents)) {
            return result;
        }

        AStarSippSolver sipp;

        for (std::size_t activeIndex = 0; activeIndex < agents.size(); activeIndex++) {
            std::unordered_set<std::string> repairFingerprints;

            while (true) {
                const SolutionConflicts currentConflicts = getCollision(result.paths);
                const std::optional<ActiveConflict> conflict = earliestExplicitConflict(
                    activeIndex,
                    result.paths,
                    currentConflicts
                );
                if (!conflict) {
                    break;
                }

                if (conflict->time == 0) {
                    synchronizeResult(
                        result,
                        grid,
                        agents,
                        initialCosts,
                        allInitialPathsExist,
                        false,
                        startedAt
                    );
                    return result;
                }

                const std::vector<Cell*> oldPath(
                    result.paths[activeIndex].begin(),
                    result.paths[activeIndex].end()
                );
                const int prefixEnd = conflict->time - 1;
                const int suffixStart = conflict->kind == ConflictKind::Vertex
                    ? conflict->time + 1
                    : conflict->time;
                const PathReservationState repairState = buildReservationState(
                    grid,
                    agents,
                    result.paths,
                    activeIndex
                );

                std::optional<std::vector<Cell*>> acceptedCandidate;
                if (
                    prefixEnd >= 0 &&
                    suffixStart >= 0 &&
                    suffixStart < static_cast<int>(oldPath.size())
                ) {
                    for (std::size_t anchor : adaptiveAnchors(prefixEnd)) {
                        std::list<Cell*> bridge = sipp.solve(
                            grid,
                            oldPath[anchor],
                            oldPath[suffixStart],
                            repairState.safeIntervalTable,
                            static_cast<int>(anchor)
                        );
                        if (bridge.empty()) {
                            continue;
                        }

                        std::optional<std::vector<Cell*>> candidate = buildLocalCandidate(
                            grid,
                            agents[activeIndex],
                            oldPath,
                            anchor,
                            suffixStart,
                            bridge,
                            result.reservations,
                            repairState
                        );
                        if (!candidate || !structurallyValid(
                            *candidate,
                            oldPath.front(),
                            oldPath.back()
                        )) {
                            continue;
                        }

                        const std::optional<int> nextConflictTime =
                            earliestConflictWithFrozenPaths(
                                *candidate,
                                activeIndex,
                                result.paths
                            );
                        if (nextConflictTime && *nextConflictTime <= conflict->time) {
                            continue;
                        }

                        const std::string fingerprint = repairFingerprint(
                            *candidate,
                            nextConflictTime
                        );
                        if (!repairFingerprints.insert(fingerprint).second) {
                            continue;
                        }

                        acceptedCandidate = std::move(candidate);
                        break;
                    }
                }

                if (!acceptedCandidate) {
                    const std::vector<std::list<Cell*>> otherPaths = pathsExcept(
                        result.paths,
                        activeIndex
                    );
                    Cell* start = grid.getCellPtr(
                        agents[activeIndex].startPosition.x,
                        agents[activeIndex].startPosition.y
                    );
                    Cell* goal = grid.getCellPtr(
                        agents[activeIndex].goalPosition.x,
                        agents[activeIndex].goalPosition.y
                    );
                    std::list<Cell*> fullPath = sipp.solve(
                        grid,
                        start,
                        goal,
                        otherPaths
                    );
                    std::vector<Cell*> fullCandidate(fullPath.begin(), fullPath.end());

                    if (
                        !structurallyValid(fullCandidate, start, goal) ||
                        earliestConflictWithFrozenPaths(
                            fullCandidate,
                            activeIndex,
                            result.paths
                        )
                    ) {
                        synchronizeResult(
                            result,
                            grid,
                            agents,
                            initialCosts,
                            allInitialPathsExist,
                            false,
                            startedAt
                        );
                        return result;
                    }

                    acceptedCandidate = std::move(fullCandidate);
                }

                std::vector<std::list<Cell*>> candidatePaths = result.paths;
                candidatePaths[activeIndex] = std::list<Cell*>(
                    acceptedCandidate->begin(),
                    acceptedCandidate->end()
                );
                PathReservationState candidateReservations = buildReservationState(
                    grid,
                    agents,
                    candidatePaths
                );

                result.paths = std::move(candidatePaths);
                result.reservations = std::move(candidateReservations);
                synchronizeResult(
                    result,
                    grid,
                    agents,
                    initialCosts,
                    allInitialPathsExist,
                    false,
                    startedAt
                );
            }
        }

        synchronizeResult(
            result,
            grid,
            agents,
            initialCosts,
            allInitialPathsExist,
            true,
            startedAt
        );
        return result;
    }

}
