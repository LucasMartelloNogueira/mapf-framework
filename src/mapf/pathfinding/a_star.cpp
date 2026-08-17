#include "mapf/pathfinding/a_star.hpp"

#include "mapf/core/a-star-heap.hpp"
#include "mapf/core/a-star-node.hpp"

#include <unordered_map>
#include <unordered_set>

namespace mapf {

    std::list<Cell*> AStarSolver::solve(Grid& grid, Cell* start, Cell* goal) {
        if (start == nullptr || goal == nullptr) {
            return {};
        }

        if (!start->isFree || !goal->isFree) {
            return {};
        }

        std::unordered_set<Cell*> closedSet;
        std::unordered_map<Cell*, int> bestG;
        std::unordered_map<Cell*, Cell*> cameFrom;
        AStarHeap openHeap;

        int firstH = grid.getManhattanDistance(start, goal);

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

            if (closedSet.contains(current.cell)) {
                continue;
            }

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

            std::list<Cell*> neighbors = grid.getNeighbors(current.cell);

            for (Cell* neighbor : neighbors) {
                if (neighbor == nullptr || !neighbor->isFree) {
                    continue;
                }

                if (closedSet.contains(neighbor)) {
                    continue;
                }

                int neighborG = current.g + 1;
                int neighborH = grid.getManhattanDistance(neighbor, goal);
                int neighborF = neighborG + neighborH;

                auto neighborBestG = bestG.find(neighbor);
                bool wasNotDiscovered = neighborBestG == bestG.end();
                bool betterPathFound = !wasNotDiscovered && neighborG < neighborBestG->second;

                if (!wasNotDiscovered && !betterPathFound) {
                    continue;
                }

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

        return {};
    }

}
