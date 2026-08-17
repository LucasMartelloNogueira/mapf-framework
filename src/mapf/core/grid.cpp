#include "mapf/core/grid.hpp"

#include <cmath>
#include <iostream>

namespace mapf {

    Grid::Grid(int rows, int cols) {
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

    Grid::Grid(std::vector<std::vector<int>>* free, int rows, int cols) {
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

    int Grid::getCellIndex(int x, int y) {
        if (x < 0 || x >= rows) {
            return -1;
        }

        if (y < 0 || y >= cols) {
            return -1;
        }

        return x * cols + y;
    }

    Cell* Grid::getCellPtr(int x, int y) {
        int index = getCellIndex(x, y);
        if (index != -1) {
            return &cells[index];
        }

        return nullptr;
    }

    std::list<Cell*> Grid::getNeighbors(Cell* cell) {
        std::list<Cell*> neighbors;

        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                if (i == 0 && j == 0) {
                    continue;
                }

                int x = cell->position.x + i;
                int y = cell->position.y + j;

                if (std::abs(i) != std::abs(j)) {
                    Cell* neighbor = getCellPtr(x, y);
                    if (neighbor != nullptr) {
                        neighbors.push_back(neighbor);
                    }
                }
            }
        }

        return neighbors;
    }

    int Grid::getManhattanDistance(Cell* a, Cell* b) {
        return std::abs(a->position.x - b->position.x) + std::abs(a->position.y - b->position.y);
    }

    void Grid::printGrid() {
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                int index = i * cols + j;
                Cell cell = cells[index];
                std::cout << "(" << cell.position.x << ", " << cell.position.y << ", " << cell.isFree << ")";
            }

            std::cout << std::endl;
        }
    }

    std::vector<Cell>& Grid::getCells() {
        return cells;
    }

    const std::vector<Cell>& Grid::getCells() const {
        return cells;
    }

}
