#include "mapf/core/grid.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace mapf {

    Grid::Grid(int rows, int cols) {
        if (rows <= 0 || cols <= 0) {
            throw std::invalid_argument("Grid dimensions must be positive.");
        }

        this->rows = rows;
        this->cols = cols;

        cells.reserve(rows * cols);

        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < cols; x++) {
                Position position {
                    .x = x,
                    .y = y
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
        if (free == nullptr) {
            throw std::invalid_argument("Free cell matrix must not be null.");
        }

        if (rows <= 0 || cols <= 0) {
            throw std::invalid_argument("Grid dimensions must be positive.");
        }

        if (static_cast<int>(free->size()) != rows) {
            throw std::invalid_argument("Free cell matrix row count does not match rows.");
        }

        for (const std::vector<int>& row : *free) {
            if (static_cast<int>(row.size()) != cols) {
                throw std::invalid_argument("Free cell matrix column count does not match cols.");
            }

            for (int value : row) {
                if (value != 0 && value != 1) {
                    throw std::invalid_argument("Free cell matrix values must be 0 or 1.");
                }
            }
        }

        this->rows = rows;
        this->cols = cols;

        cells.reserve(rows * cols);

        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < cols; x++) {
                int matrixRow = rows - 1 - y;
                Position position {
                    .x = x,
                    .y = y
                };

                int isFree = (*free)[matrixRow][x];

                Cell cell {
                    .position = position,
                    .isFree = isFree == 1
                };

                this->cells.push_back(cell);
            }
        }
    }

    int Grid::getCellIndex(int x, int y) {
        if (x < 0 || x >= cols) {
            return -1;
        }

        if (y < 0 || y >= rows) {
            return -1;
        }

        return y * cols + x;
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
        for (int y = rows - 1; y >= 0; y--) {
            for (int x = 0; x < cols; x++) {
                int index = y * cols + x;
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
