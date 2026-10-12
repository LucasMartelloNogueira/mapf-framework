#include "mapf/pathfinding/a_star_sipp.hpp"

#include "mapf/pathfinding/sipp/compare_sipp_node.hpp"
#include "mapf/pathfinding/sipp/parent_info.hpp"
#include "mapf/pathfinding/sipp/sipp_node.hpp"
#include "mapf/pathfinding/sipp/state_key_hash.hpp"

#include <algorithm>
#include <queue>
#include <span>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace mapf {

    namespace {

        // diminui o numero de intervalos
        // junta intervalos que tem sobreposição de tempo
        void mergeCollisionIntervals(std::vector<Interval>& intervals) {
            if (intervals.empty()) {
                return;
            }

            std::sort(
                intervals.begin(),
                intervals.end(),
                [](const Interval& a, const Interval& b) {
                    if (a.start != b.start) {
                        return a.start < b.start;
                    }

                    return a.end < b.end;
                }
            );

            std::vector<Interval> merged;
            merged.push_back(intervals[0]);

            for (int i = 1; i < static_cast<int>(intervals.size()); i++) {
                Interval& last = merged.back();
                const Interval& current = intervals[i];

                if (current.start <= last.end + 1) {
                    last.end = std::max(last.end, current.end);
                } else {
                    merged.push_back(current);
                }
            }

            intervals = merged;
        }

        // retorna o intervalo de tempo para uma celula em um instante de tempo
        // Only the key participates in hashing. Search metadata can change in
        // place without invalidating an unordered_set entry.
        struct SearchState {
            StateKey key;
            mutable int bestG = SAFE_INTERVAL_INFINITY;
            mutable ParentInfo parent {};
            mutable bool closed = false;

            bool operator==(const SearchState& other) const { return key == other.key; }
        };

        struct SearchStateHash {
            std::size_t operator()(const SearchState& state) const {
                return StateKeyHash{}(state.key);
            }
        };

        std::span<const Interval> intervalsForCell(const SafeIntervalsByCell& table, Cell* cell) {
            static constexpr Interval unrestricted {0, SAFE_INTERVAL_INFINITY};
            const auto entry = table.find(cell);
            // An explicitly empty vector means blocked at every time.
            return entry == table.end() ? std::span<const Interval>(&unrestricted, 1)
                : std::span<const Interval>(entry->second);
        }

        int findIntervalIndexAtTime(const SafeIntervalsByCell& table, Cell* cell, int time) {
            const auto intervals = intervalsForCell(table, cell);
            for (std::size_t i = 0; i < intervals.size(); ++i) {
                if (intervals[i].start <= time && time <= intervals[i].end) {
                    return static_cast<int>(i);
                }
            }
            return -1;
        }

        bool isEdgeBlocked(
            const BlockedEdgeArrivals& blockedEdgeArrivals,
            Cell* from,
            Cell* to,
            int arrivalTime
        ) {
            auto it = blockedEdgeArrivals.find({from, to});
            if (it == blockedEdgeArrivals.end()) {
                return false;
            }

            return it->second.contains(arrivalTime);
        }
    }

    SafeIntervalTable AStarSippSolver::getSafeIntervalsByCell(
        Grid&,
        const std::vector<std::list<Cell*>>& otherAgentPaths
    ) {
        std::unordered_map<Cell*, std::vector<Interval>> collisionIntervalsByCell;
        SafeIntervalTable table;

        for (const std::list<Cell*>& pathList : otherAgentPaths) {
            if (pathList.empty()) {
                continue;
            }

            std::vector<Cell*> path(pathList.begin(), pathList.end());

            for (int t = 0; t < static_cast<int>(path.size()); t++) {
                Cell* occupiedCell = path[t];
                collisionIntervalsByCell[occupiedCell].push_back({t, t});

                if (t > 0) {
                    Cell* previousCell = path[t - 1];
                    if (previousCell != occupiedCell) {
                        table.blockedEdgeArrivals[{occupiedCell, previousCell}].insert(t);
                    }
                }
            }

            Cell* goalCell = path.back();
            int lastTime = static_cast<int>(path.size()) - 1;
            collisionIntervalsByCell[goalCell].push_back({lastTime, SAFE_INTERVAL_INFINITY});
        }

        for (auto& [cellPtr, collisionIntervals] : collisionIntervalsByCell) {
            mergeCollisionIntervals(collisionIntervals);

            std::vector<Interval> safeIntervals;
            int currentStart = 0;

            for (const Interval& collision : collisionIntervals) {
                if (currentStart < collision.start) {
                    safeIntervals.push_back({currentStart, collision.start - 1});
                }

                if (collision.end >= SAFE_INTERVAL_INFINITY) {
                    currentStart = SAFE_INTERVAL_INFINITY;
                    break;
                }

                currentStart = collision.end + 1;
            }

            if (currentStart < SAFE_INTERVAL_INFINITY) {
                safeIntervals.push_back({currentStart, SAFE_INTERVAL_INFINITY});
            }

            table.safeIntervalsByCell.emplace(cellPtr, std::move(safeIntervals));
        }

        return table;
    }

    std::list<Cell*> AStarSippSolver::solve(
        Grid& grid,
        Cell* start,
        Cell* goal,
        const std::vector<std::list<Cell*>>& otherAgentPaths
    ) {
        SafeIntervalTable intervalTable = getSafeIntervalsByCell(grid, otherAgentPaths);
        return solveWithTable(
            grid,
            start,
            goal,
            intervalTable,
            0,
            GoalOccupation::Permanent
        );
    }

    std::list<Cell*> AStarSippSolver::solve(
        Grid& grid,
        Cell* start,
        Cell* goal,
        const SafeIntervalTable& safeIntervalTable,
        int startTime
    ) {
        return solve(
            grid,
            start,
            goal,
            safeIntervalTable,
            startTime,
            GoalOccupation::Transient
        );
    }

    std::list<Cell*> AStarSippSolver::solve(
        Grid& grid,
        Cell* start,
        Cell* goal,
        const SafeIntervalTable& safeIntervalTable,
        int startTime,
        GoalOccupation goalOccupation
    ) {
        switch (goalOccupation) {
        case GoalOccupation::Transient:
        case GoalOccupation::Permanent:
            return solveWithTable(grid, start, goal, safeIntervalTable, startTime, goalOccupation);
        default:
            throw std::invalid_argument("Unknown SIPP goal occupation policy.");
        }
    }

    std::list<Cell*> AStarSippSolver::solveWithTable(
        Grid& grid,
        Cell* start,
        Cell* goal,
        const SafeIntervalTable& intervalTable,
        int startTime,
        GoalOccupation goalOccupation
    ) {
        if (start == nullptr || goal == nullptr) {
            return {};
        }

        if (!start->isFree || !goal->isFree || startTime < 0) {
            return {};
        }

        int startIntervalIndex = findIntervalIndexAtTime(
            intervalTable.safeIntervalsByCell,
            start,
            startTime
        );
        if (startIntervalIndex == -1) {
            return {};
        }

        std::priority_queue<SippNode, std::vector<SippNode>, CompareSippNode> openHeap;
        std::unordered_set<SearchState, SearchStateHash> states;

        StateKey startKey {
            .cell = start,
            .intervalIndex = startIntervalIndex
        };

        int startH = grid.getManhattanDistance(start, goal);

        openHeap.push({
            .key = startKey,
            .time = startTime,
            .g = startTime,
            .h = startH,
            .f = startTime + startH
        });

        states.insert({startKey, startTime, {startKey, false}, false});

        while (!openHeap.empty()) {
            SippNode current = openHeap.top();
            openHeap.pop();

            const auto currentState = states.find({current.key});
            if (currentState == states.end() || current.g != currentState->bestG || currentState->closed) {
                continue;
            }

            const auto currentIntervals = intervalsForCell(intervalTable.safeIntervalsByCell, current.key.cell);
            if (
                current.key.intervalIndex < 0 ||
                current.key.intervalIndex >= static_cast<int>(currentIntervals.size())
            ) {
                continue;
            }

            const Interval& selectedInterval = currentIntervals[current.key.intervalIndex];

            // aqui mostra a necessidade de GoalOccupation::Transient e GoalOccupation::Permanent
            // Permanent: usado para SIPP normal
            // Transient: usado no SIPP do reparo local
            // se for para chegar no destino: tem que ser em um intervalo de tempo no qual o final é infinito, para não ter colisão depois
            // se for para chegar no destino no reparo local: pode chegar em qualquer intervalo de tempo valido, pois depois o agente vai sair
            const bool goalPolicySatisfied =
                goalOccupation == GoalOccupation::Transient ||
                selectedInterval.end >= SAFE_INTERVAL_INFINITY;

            if (current.key.cell == goal && goalPolicySatisfied) {
                std::vector<StateKey> stateSequence;
                StateKey traceKey = current.key;

                while (true) {
                    stateSequence.push_back(traceKey);

                    const ParentInfo& parentInfo = states.find({traceKey})->parent;
                    if (!parentInfo.hasParent) {
                        break;
                    }

                    traceKey = parentInfo.parent;
                }

                std::reverse(stateSequence.begin(), stateSequence.end());

                std::list<Cell*> path;
                path.push_back(stateSequence[0].cell);

                for (int i = 1; i < static_cast<int>(stateSequence.size()); i++) {
                    StateKey parentKey = stateSequence[i - 1];
                    StateKey childKey = stateSequence[i];

                    int parentTime = states.find({parentKey})->bestG;
                    int childTime = states.find({childKey})->bestG;

                    for (int t = parentTime + 1; t < childTime; t++) {
                        path.push_back(parentKey.cell);
                    }

                    path.push_back(childKey.cell);
                }

                return path;
            }

            currentState->closed = true;

            const Interval& currentInterval = selectedInterval;
            std::list<Cell*> neighbors = grid.getNeighbors(current.key.cell);

            for (Cell* neighbor : neighbors) {
                if (neighbor == nullptr || !neighbor->isFree) {
                    continue;
                }

                const auto neighborIntervals = intervalsForCell(intervalTable.safeIntervalsByCell, neighbor);

                for (int neighborIntervalIndex = 0; neighborIntervalIndex < static_cast<int>(neighborIntervals.size()); neighborIntervalIndex++) {
                    const Interval& neighborInterval = neighborIntervals[neighborIntervalIndex];

                    int startArrivalTime = current.time + 1;
                    int endArrivalTime = currentInterval.end >= SAFE_INTERVAL_INFINITY
                        ? SAFE_INTERVAL_INFINITY
                        : currentInterval.end + 1;

                    if (neighborInterval.start > endArrivalTime || neighborInterval.end < startArrivalTime) {
                        continue;
                    }

                    int earliestArrivalTime = std::max(startArrivalTime, neighborInterval.start);
                    int latestArrivalTime = std::min(endArrivalTime, neighborInterval.end);

                    while (
                        earliestArrivalTime <= latestArrivalTime &&
                        isEdgeBlocked(intervalTable.blockedEdgeArrivals, current.key.cell, neighbor, earliestArrivalTime)
                    ) {
                        earliestArrivalTime++;
                    }

                    if (earliestArrivalTime > latestArrivalTime) {
                        continue;
                    }

                    StateKey neighborKey {
                        .cell = neighbor,
                        .intervalIndex = neighborIntervalIndex
                    };

                    auto neighborState = states.find({neighborKey});
                    bool wasNotDiscovered = neighborState == states.end();
                    bool betterPathFound = !wasNotDiscovered && earliestArrivalTime < neighborState->bestG;

                    if (!wasNotDiscovered && !betterPathFound) {
                        continue;
                    }

                    int neighborH = grid.getManhattanDistance(neighbor, goal);

                    openHeap.push({
                        .key = neighborKey,
                        .time = earliestArrivalTime,
                        .g = earliestArrivalTime,
                        .h = neighborH,
                        .f = earliestArrivalTime + neighborH
                    });

                    if (wasNotDiscovered) {
                        states.insert({neighborKey, earliestArrivalTime, {current.key, true}, false});
                    } else {
                        neighborState->bestG = earliestArrivalTime;
                        neighborState->parent = {current.key, true};
                    }
                }
            }
        }

        return {};
    }
}
