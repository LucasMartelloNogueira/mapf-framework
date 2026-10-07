#pragma once

#include <filesystem>
#include <list>
#include <string>
#include <variant>
#include <vector>

#include "mapf/core/cell.hpp"
#include "mapf/core/solution_conflicts.hpp"

void printPath(const std::list<mapf::Cell*>& path);


// All cells must belong to one live grid. Rejects null cells, duplicate nonempty
// starts/goals and unrepresentable path indexes/times with std::invalid_argument.
// Empty path slots stay aligned; global events include virtual destination owners.
mapf::SolutionConflicts getCollision(const std::vector<std::list<mapf::Cell*>>& paths);

// Updates explicit path memberships and removes exactly one record from byAgent[index].
// Event entries are retained even when they no longer represent an active conflict.
// Other agents' records and virtual goal occupancy are not recalculated.
mapf::SolutionConflicts updateSolutionConflicts(
    std::list<mapf::Cell*> oldPath,
    std::list<mapf::Cell*> newPath,
    int index,
    std::variant<mapf::CellConflict, mapf::EdgeConflict> conflict,
    mapf::SolutionConflicts conflicts
);


// Collision predicate only: empty slots do not imply failure. Invalid detector
// input returns false; callers must check required path completeness separately.
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
