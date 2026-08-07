#pragma once

#include <vector>
#include <iostream>
#include <format>
#include <string>
#include <list>
#include <cmath>
#include <unordered_set>
#include <unordered_map>

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

    };

}
