#pragma once

#include <vector>
#include <iostream>
#include <format>
#include <string>
#include <list>
#include <cmath>
#include <unordered_set>
#include <unordered_map>
#include <limits>
#include <algorithm>

#include "cell.hpp"
#include "a-star-node.hpp"
#include "a-star-heap.hpp"

namespace mapf {

    class Grid {
        private:
            std::vector<Cell> cells;
            int rows;
            int cols;

        public:
            Grid(int rows, int cols) {
                this->rows = rows;
                this->cols = cols;

                cells.reserve(rows * cols);

                for (int i = 0; i < rows; i++) {
                    for (int j = 0; j < cols; j++) {

                        Position position {
                            .x = i,
                            .y = j
                        };

                        Cell cell {
                            .position = position,
                            .isFree = true
                        };

                        this->cells.push_back(cell);
                    }
                }
            }

            
            Grid(std::vector<std::vector<int>>* free, int rows, int cols) {
                this->rows = rows;
                this->cols = cols;

                cells.reserve(rows * cols);

                for (int i = 0; i < rows; i++) {
                    for (int j = 0; j < cols; j++) {
                        Position position {
                            .x = i,
                            .y = j
                        };

                        int isFree = (*free)[i][j];

                        Cell cell {
                            .position = position,
                            .isFree = isFree == 1
                        };

                        this->cells.push_back(cell);
                    }
                }
            }


            int getCellIndex(int x, int y) {

                if (x < 0 || x >= rows) {
                    return -1;
                }

                if (y < 0 || y >= cols) {
                    return -1;
                }

                return x * cols + y;
            }

            
            Cell* getCellPtr(int x, int y) {
                int index = getCellIndex(x, y);
                if (index != -1) {
                    Cell* cellPtr = &cells[index];
                    return cellPtr;
                }

                return NULL;
            }

            
            std::list<Cell*> getNeighbors(Cell* cell) {

                std::list<Cell*> neighbors;

                for (int i = -1; i<= 1; i++) {
                    for (int j = -1; j <= 1; j++) {

                        if (i == 0 && j == 0) {
                            continue;
                        }

                        int x = cell->position.x + i;
                        int y = cell->position.y + j;

                        if (std::abs(i) != std::abs(j)) {
                            Cell* neighbor = getCellPtr(x, y);
                            if (neighbor != NULL) {
                                neighbors.push_back(neighbor);
                            } 
                        }
                    }
                }

                return neighbors;
            }


            int getManhattanDistance(Cell* a, Cell* b) {
                return abs(a->position.x - b->position.x) + abs(a->position.y - b->position.y);
            }


            void printGrid() {
                for (int i = 0; i < rows; i++) {
                    for (int j = 0; j < cols; j++) {
                        
                        int index = i * cols + j;
                        Cell cell = cells[index];
                        std::cout << "(" << cell.position.x << ", " << cell.position.y << ", " << cell.isFree << ")";
                    }

                    std::cout << std::endl;
                }
            }


            std::list<Cell*> getAStarPath(Cell* start, Cell* goal) {

                /*
                A* usando estratégia de lazy deletion

                lazy deletion: quando um nó, que está em open, é encontrado com um caminho melhor (g menor)
                               esse nó não é atualizado em open. O Nó com melhor caminho é inserido na heap
                               e é marcado como melhor (por meio do bestG). Assim, o nó com o pior caminho 
                               é desconsiderado quando é removido de open. Isso evita que tenhamos que iterar
                               em open para atualizar o caminho do nó
                */

                if (start == nullptr || goal == nullptr) {
                    return {};
                }
                
                if (!start->isFree || !goal->isFree) {
                    return {};
                }

                std::unordered_set<Cell*> closedSet;

                // usado para lazy deletion
                std::unordered_map<Cell*, int> bestG;

                // necessário para calcular o caminho final
                // necessário pois temos Cell e AStarNode
                // a função de pegar vizinhos pega só Cells
                // mas o algortimo usa AStartNode para itens de open
                // então precisamos dessa estrutura para guardar de onde um nó veio
                // key: filho / value: pai
                std::unordered_map<Cell*, Cell*> cameFrom;
                AStarHeap openHeap;
                
                int firstH = getManhattanDistance(start, goal);

                AStarNode first{
                    .cell = start,
                    .parent = nullptr,
                    .g = 0,
                    .h = firstH,
                    .f = firstH,
                };
                
                bestG[first.cell] = 0;
                cameFrom[first.cell] = nullptr;
                openHeap.push(first);

                while (openHeap.size() > 0) {
                    AStarNode current = openHeap.top();
                    openHeap.pop();
                    
                    auto currentBestG = bestG.find(current.cell);

                    if (
                        currentBestG == bestG.end() ||
                        current.g != currentBestG->second
                    ) {
                        continue;
                    }

                    // TODO: ver erro
                    if (closedSet.contains(current.cell)) {
                        continue;
                    }


                    //verificando se chegou no destino
                    if (current.cell == goal) {
                        std::list<Cell*> path;
                        Cell* pathCell = goal;

                        while (pathCell != nullptr) {
                            path.push_front(pathCell);
                            pathCell = cameFrom[pathCell];
                        }

                        return path;
                    }
                    closedSet.insert(current.cell);


                    std::list<Cell*>neighbors = getNeighbors(current.cell);

                    for (Cell* neighbor : neighbors) {

                        if (neighbor == nullptr || !neighbor->isFree) {
                            continue;
                        }

                        // TODO: ver erro
                        if (closedSet.contains(neighbor)) {
                            continue;
                        }

                        int neighborG = current.g + 1;
                        int neighborH = getManhattanDistance(neighbor, goal);
                        int neighborF = neighborG + neighborH;

                        auto neighborBestG = bestG.find(neighbor);
                        bool wasNotDiscovered = neighborBestG == bestG.end();
                        bool betterPathFound = !wasNotDiscovered && neighborG < neighborBestG->second;

                        // se o nó já foi explorado e não tem caminho melhor, ignora
                        if (!wasNotDiscovered && !betterPathFound) {
                            continue;
                        }

                        // aqui significa que achamos um nó que não foi explorado ou que tem caminho melhor
                        AStarNode neighborNode {
                            .cell = neighbor,
                            .parent = current.cell,
                            .g = neighborG,
                            .h = neighborH,
                            .f = neighborF
                        };

                        bestG[neighbor] = neighborG;
                        cameFrom[neighbor] = current.cell;
                        openHeap.push(neighborNode);
                    }
                }

                // não achou caminho
                return {};
            }


            std::list<Cell*> getSippPath(Cell* start, Cell* goal, std::vector<std::list<Cell*>> otherAgentPaths) {
                if (start == nullptr || goal == nullptr) {
                    return {};
                }

                if (!start->isFree || !goal->isFree) {
                    return {};
                }

                constexpr int INF_TIME = std::numeric_limits<int>::max() / 4;

                struct Interval {
                    int start;
                    int end;
                };

                struct EdgeKey {
                    Cell* from;
                    Cell* to;

                    bool operator==(const EdgeKey& other) const {
                        return from == other.from && to == other.to;
                    }
                };

                struct EdgeKeyHash {
                    std::size_t operator()(const EdgeKey& key) const {
                        std::size_t fromHash = std::hash<Cell*>{}(key.from);
                        std::size_t toHash = std::hash<Cell*>{}(key.to);
                        return fromHash ^ (toHash << 1);
                    }
                };

                struct StateKey {
                    Cell* cell;
                    int intervalIndex;

                    bool operator==(const StateKey& other) const {
                        return cell == other.cell && intervalIndex == other.intervalIndex;
                    }
                };

                struct StateKeyHash {
                    std::size_t operator()(const StateKey& key) const {
                        std::size_t cellHash = std::hash<Cell*>{}(key.cell);
                        std::size_t intervalHash = std::hash<int>{}(key.intervalIndex);
                        return cellHash ^ (intervalHash << 1);
                    }
                };

                struct SippNode {
                    StateKey key;
                    int time;
                    int g;
                    int h;
                    int f;
                };

                struct CompareSippNode {
                    bool operator()(const SippNode& a, const SippNode& b) const {
                        if (a.f != b.f) {
                            return a.f > b.f;
                        }

                        return a.g > b.g;
                    }
                };

                struct ParentInfo {
                    StateKey parent;
                    bool hasParent;
                };

                std::unordered_map<Cell*, std::vector<Interval>> collisionIntervalsByCell;
                std::unordered_map<EdgeKey, std::unordered_set<int>, EdgeKeyHash> blockedEdgeArrivals;

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
                            blockedEdgeArrivals[{occupiedCell, previousCell}].insert(t);
                        }
                    }

                    Cell* goalCell = path.back();
                    int lastTime = static_cast<int>(path.size()) - 1;
                    collisionIntervalsByCell[goalCell].push_back({lastTime, INF_TIME});
                }

                auto mergeCollisionIntervals = [](std::vector<Interval>& intervals) {
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
                };

                std::unordered_map<Cell*, std::vector<Interval>> safeIntervalsByCell;
                safeIntervalsByCell.reserve(cells.size());

                for (Cell& cell : cells) {
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

                    safeIntervalsByCell[cellPtr] = safeIntervals;
                }

                auto findIntervalIndexAtTime = [&](Cell* cell, int time) -> int {
                    const std::vector<Interval>& intervals = safeIntervalsByCell[cell];

                    for (int i = 0; i < static_cast<int>(intervals.size()); i++) {
                        if (intervals[i].start <= time && time <= intervals[i].end) {
                            return i;
                        }
                    }

                    return -1;
                };

                auto isEdgeBlocked = [&](Cell* from, Cell* to, int arrivalTime) -> bool {
                    auto it = blockedEdgeArrivals.find({from, to});
                    if (it == blockedEdgeArrivals.end()) {
                        return false;
                    }

                    return it->second.contains(arrivalTime);
                };

                int startIntervalIndex = findIntervalIndexAtTime(start, 0);
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

                int startH = getManhattanDistance(start, goal);

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

                    const Interval& currentInterval = safeIntervalsByCell[current.key.cell][current.key.intervalIndex];
                    std::list<Cell*> neighbors = getNeighbors(current.key.cell);

                    for (Cell* neighbor : neighbors) {
                        if (neighbor == nullptr || !neighbor->isFree) {
                            continue;
                        }

                        const std::vector<Interval>& neighborIntervals = safeIntervalsByCell[neighbor];

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
                                isEdgeBlocked(current.key.cell, neighbor, earliestArrivalTime)
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

                            int neighborH = getManhattanDistance(neighbor, goal);

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
            
    };

}
