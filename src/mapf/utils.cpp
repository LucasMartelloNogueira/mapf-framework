#include "mapf/utils.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {
    bool cellComesBefore(const mapf::Cell* first, const mapf::Cell* second) {
        if (first->position.x != second->position.x) {
            return first->position.x < second->position.x;
        }

        return first->position.y < second->position.y;
    }

    mapf::Cell* positionAt(const std::vector<mapf::Cell*>& path, int time) {
        if (time < static_cast<int>(path.size())) {
            return path[time];
        }

        return path.back();
    }

    std::string escapeCsvField(const std::string& field) {
        bool mustQuote = field.find_first_of(",\"\r\n") != std::string::npos;
        if (!mustQuote) {
            return field;
        }

        std::string escaped = "\"";

        for (char character : field) {
            if (character == '"') {
                escaped += "\"\"";
            } else {
                escaped += character;
            }
        }

        escaped += "\"";
        return escaped;
    }

    void writeCsvRow(std::ofstream& output, const CsvRow& fields) {
        bool first = true;

        for (const std::string& field : fields) {
            if (!first) {
                output << ",";
            }

            output << escapeCsvField(field);
            first = false;
        }

        output << "\n";
    }
}

void printPath(const std::list<mapf::Cell*>& path) {
    for (mapf::Cell* cell : path) {
        mapf::printCell(cell);
    }
}

bool validateSolution(const std::vector<std::list<mapf::Cell*>>& paths) {
    mapf::SolutionConflicts conflicts = getCollision(paths);
    return conflicts.cellConflicts.empty() && conflicts.edgeConflicts.empty();
}

mapf::SolutionConflicts getCollision(const std::vector<std::list<mapf::Cell*>>& paths) {
    mapf::SolutionConflicts conflicts;
    std::vector<std::vector<mapf::Cell*>> indexedPaths;
    indexedPaths.reserve(paths.size());

    int makespan = 0;
    for (const std::list<mapf::Cell*>& path : paths) {
        indexedPaths.emplace_back(path.begin(), path.end());
        if (!path.empty()) {
            makespan = std::max(makespan, static_cast<int>(path.size()) - 1);
        }
    }

    for (int time = 0; time <= makespan; time++) {
        for (std::size_t first = 0; first < indexedPaths.size(); first++) {
            if (indexedPaths[first].empty()) {
                continue;
            }

            for (std::size_t second = first + 1; second < indexedPaths.size(); second++) {
                if (indexedPaths[second].empty()) {
                    continue;
                }

                mapf::Cell* firstCell = positionAt(indexedPaths[first], time);
                mapf::Cell* secondCell = positionAt(indexedPaths[second], time);

                if (firstCell == secondCell) {
                    conflicts.cellConflicts.push_back({*firstCell, time});
                }
            }
        }

        if (time == 0) {
            continue;
        }

        for (std::size_t first = 0; first < indexedPaths.size(); first++) {
            if (indexedPaths[first].empty()) {
                continue;
            }

            for (std::size_t second = first + 1; second < indexedPaths.size(); second++) {
                if (indexedPaths[second].empty()) {
                    continue;
                }

                mapf::Cell* firstPrevious = positionAt(indexedPaths[first], time - 1);
                mapf::Cell* firstCurrent = positionAt(indexedPaths[first], time);
                mapf::Cell* secondPrevious = positionAt(indexedPaths[second], time - 1);
                mapf::Cell* secondCurrent = positionAt(indexedPaths[second], time);

                const bool firstMoved = firstPrevious != firstCurrent;
                const bool secondMoved = secondPrevious != secondCurrent;
                const bool oppositeDirections =
                    firstPrevious == secondCurrent &&
                    firstCurrent == secondPrevious;

                if (firstMoved && secondMoved && oppositeDirections) {
                    mapf::Cell* edgeFirst = firstPrevious;
                    mapf::Cell* edgeSecond = firstCurrent;
                    if (cellComesBefore(edgeSecond, edgeFirst)) {
                        std::swap(edgeFirst, edgeSecond);
                    }

                    conflicts.edgeConflicts.push_back({*edgeFirst, *edgeSecond, time});
                }
            }
        }
    }

    return conflicts;
}




bool writeRowsToCsvFile(
    const std::filesystem::path& outputPath,
    const CsvRow& headers,
    const std::vector<CsvRow>& rows
) {
    if (outputPath.extension() != ".csv") {
        return false;
    }

    for (const CsvRow& row : rows) {
        if (headers.size() != row.size()) {
            return false;
        }
    }

    std::filesystem::path parentPath = outputPath.parent_path();
    if (!parentPath.empty()) {
        std::error_code errorCode;
        std::filesystem::create_directories(parentPath, errorCode);
        if (errorCode) {
            return false;
        }
    }

    std::ofstream output(outputPath, std::ios::trunc);
    if (!output.is_open()) {
        return false;
    }

    writeCsvRow(output, headers);
    for (const CsvRow& row : rows) {
        writeCsvRow(output, row);
    }

    return output.good();
}

bool writeResultsToCsvFile(
    const std::string& outputFilename,
    const std::list<std::string>& headers,
    const std::list<std::string>& rowValues
) {
    return writeRowsToCsvFile(
        std::filesystem::path(outputFilename),
        CsvRow(headers.begin(), headers.end()),
        {CsvRow(rowValues.begin(), rowValues.end())}
    );
}
