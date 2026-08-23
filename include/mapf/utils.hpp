#pragma once

#include <list>
#include <vector>
#include <string>

#include "mapf/core/cell.hpp"

void printPath(const std::list<mapf::Cell*>& path);


bool validateSolution(const std::vector<std::list<mapf::Cell*>>& paths);


bool writeResultsToCsvFile(
    const std::string& outputFilename,
    const std::list<std::string>& headers,
    const std::list<std::string>& rowValues
);
