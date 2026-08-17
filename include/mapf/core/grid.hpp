#pragma once

#include <list>
#include <vector>

#include "cell.hpp"

namespace mapf {

    class Grid {
        private:
            std::vector<Cell> cells;
            int rows;
            int cols;

        public:
            Grid(int rows, int cols);
            Grid(std::vector<std::vector<int>>* free, int rows, int cols);

            int getCellIndex(int x, int y);
            Cell* getCellPtr(int x, int y);
            std::list<Cell*> getNeighbors(Cell* cell);
            int getManhattanDistance(Cell* a, Cell* b);
            void printGrid();

            std::vector<Cell>& getCells();
            const std::vector<Cell>& getCells() const;
    };

}
