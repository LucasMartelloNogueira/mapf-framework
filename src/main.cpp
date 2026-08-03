#include <iostream>

#include "mapf/core/grid.hpp"

int main() {
    
    int rows = 2;
    int cols = 2;

    std::cout << "main" << std::endl;

    mapf::Grid grid(rows, cols);
    grid.printGrid();

    return 0;
}

