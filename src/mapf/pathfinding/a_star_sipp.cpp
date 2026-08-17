#include "mapf/pathfinding/a_star_sipp.hpp"

#include "mapf/pathfinding/sipp/compare_sipp_node.hpp"
#include "mapf/pathfinding/sipp/parent_info.hpp"
#include "mapf/pathfinding/sipp/sipp_node.hpp"
#include "mapf/pathfinding/sipp/state_key_hash.hpp"

#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace mapf {

    namespace {
        constexpr int INF_TIME = std::numeric_limits<int>::max() / 4;

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

        int findIntervalIndexAtTime(const SafeIntervalsByCell& safeIntervalsByCell, Cell* cell, int time) {
            auto it = safeIntervalsByCell.find(cell);
            if (it == safeIntervalsByCell.end()) {
                return -1;
            }

            const std::vector<Interval>& intervals = it->second;

            for (int i = 0; i < static_cast<int>(intervals.size()); i++) {
                if (intervals[i].start <= time && time <= intervals[i].end) {
                    return i;
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
        Grid& grid,
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
                    table.blockedEdgeArrivals[{occupiedCell, previousCell}].insert(t);
                }
            }

            Cell* goalCell = path.back();
            int lastTime = static_cast<int>(path.size()) - 1;
            collisionIntervalsByCell[goalCell].push_back({lastTime, INF_TIME});
        }

        table.safeIntervalsByCell.reserve(grid.getCells().size());

        for (Cell& cell : grid.getCells()) {
            Cell* cellPtr = &cell;
            std::vector<Interval> collisionIntervals = collisionIntervalsByCell[cellPtr];
            mergeCollisionIntervals(collisionIntervals);

            std::vector<Interval> safeIntervals;
            int currentStart = 0;

            for (const Interval& collision : collisionIntervals) {
                if (currentStart < collision.start) {
                    safeIntervals.push_back({currentStart, collision.start - 1});
                }

                if (collision.end >= INF_TIME) {
                    currentStart = INF_TIME;
                    break;
                }

                currentStart = collision.end + 1;
            }

            if (currentStart < INF_TIME) {
                safeIntervals.push_back({currentStart, INF_TIME});
            }

            table.safeIntervalsByCell[cellPtr] = safeIntervals;
        }

        return table;
    }

    std::list<Cell*> AStarSippSolver::solve(
        Grid& grid,
        Cell* start,
        Cell* goal,
        const std::vector<std::list<Cell*>>& otherAgentPaths
    ) {
        if (start == nullptr || goal == nullptr) {
            return {};
        }

        if (!start->isFree || !goal->isFree) {
            return {};
        }

        SafeIntervalTable intervalTable = getSafeIntervalsByCell(grid, otherAgentPaths);

        int startIntervalIndex = findIntervalIndexAtTime(intervalTable.safeIntervalsByCell, start, 0);
        if (startIntervalIndex == -1) {
            return {};
        }

        std::priority_queue<SippNode, std::vector<SippNode>, CompareSippNode> openHeap;
        std::unordered_map<StateKey, int, StateKeyHash> bestG;
        std::unordered_map<StateKey, ParentInfo, StateKeyHash> cameFrom;
        std::unordered_set<StateKey, StateKeyHash> closedSet;

        StateKey startKey {
            .cell = start,
            .intervalIndex = startIntervalIndex
        };

        int startH = grid.getManhattanDistance(start, goal);

        openHeap.push({
            .key = startKey,
            .time = 0,
            .g = 0,
            .h = startH,
            .f = startH
        });

        bestG[startKey] = 0;
        cameFrom[startKey] = {
            .parent = startKey,
            .hasParent = false
        };

        while (!openHeap.empty()) {
            SippNode current = openHeap.top();
            openHeap.pop();

            auto currentBestG = bestG.find(current.key);
            if (currentBestG == bestG.end() || current.g != currentBestG->second) {
                continue;
            }

            if (closedSet.contains(current.key)) {
                continue;
            }

            if (current.key.cell == goal) {
                std::vector<StateKey> stateSequence;
                StateKey traceKey = current.key;

                while (true) {
                    stateSequence.push_back(traceKey);

                    ParentInfo parentInfo = cameFrom[traceKey];
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

                    int parentTime = bestG[parentKey];
                    int childTime = bestG[childKey];

                    for (int t = parentTime + 1; t < childTime; t++) {
                        path.push_back(parentKey.cell);
                    }

                    path.push_back(childKey.cell);
                }

                return path;
            }

            closedSet.insert(current.key);

            const Interval& currentInterval = intervalTable.safeIntervalsByCell[current.key.cell][current.key.intervalIndex];
            std::list<Cell*> neighbors = grid.getNeighbors(current.key.cell);

            for (Cell* neighbor : neighbors) {
                if (neighbor == nullptr || !neighbor->isFree) {
                    continue;
                }

                const std::vector<Interval>& neighborIntervals = intervalTable.safeIntervalsByCell[neighbor];

                for (int neighborIntervalIndex = 0; neighborIntervalIndex < static_cast<int>(neighborIntervals.size()); neighborIntervalIndex++) {
                    const Interval& neighborInterval = neighborIntervals[neighborIntervalIndex];

                    int startArrivalTime = current.time + 1;
                    int endArrivalTime = currentInterval.end >= INF_TIME ? INF_TIME : currentInterval.end + 1;

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

                    auto neighborBestG = bestG.find(neighborKey);
                    bool wasNotDiscovered = neighborBestG == bestG.end();
                    bool betterPathFound = !wasNotDiscovered && earliestArrivalTime < neighborBestG->second;

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

                    bestG[neighborKey] = earliestArrivalTime;
                    cameFrom[neighborKey] = {
                        .parent = current.key,
                        .hasParent = true
                    };
                }
            }
        }

        return {};
    }

}
