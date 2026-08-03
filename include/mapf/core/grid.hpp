#pragma once

#include <vector>
#include <iostream>
#include <format>
#include <string>

#include "cell.hpp"

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

    };

}
