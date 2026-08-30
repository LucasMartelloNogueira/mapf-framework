#pragma once

#include <list>
#include <vector>
#include <string>

#include "mapf/core/cell.hpp"
#include "mapf/core/solution_conflicts.hpp"

void printPath(const std::list<mapf::Cell*>& path);


mapf::SolutionConflicts getCollision(const std::vector<std::list<mapf::Cell*>>& paths);


bool validateSolution(const std::vector<std::list<mapf::Cell*>>& paths);


bool writeResultsToCsvFile(
    const std::string& outputFilename,
    const std::list<std::string>& headers,
    const std::list<std::string>& rowValues
);
