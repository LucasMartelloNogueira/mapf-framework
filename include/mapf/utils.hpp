#pragma once

#include <filesystem>
#include <list>
#include <string>
#include <vector>

#include "mapf/core/cell.hpp"
#include "mapf/core/solution_conflicts.hpp"

void printPath(const std::list<mapf::Cell*>& path);


mapf::SolutionConflicts getCollision(const std::vector<std::list<mapf::Cell*>>& paths);


bool validateSolution(const std::vector<std::list<mapf::Cell*>>& paths);

using CsvRow = std::vector<std::string>;

bool writeRowsToCsvFile(
    const std::filesystem::path& outputFilename,
    const CsvRow& headers,
    const std::vector<CsvRow>& rows
);

bool writeResultsToCsvFile(
    const std::string& outputFilename,
    const std::list<std::string>& headers,
    const std::list<std::string>& rowValues
);
