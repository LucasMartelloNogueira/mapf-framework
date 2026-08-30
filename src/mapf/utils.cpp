#include "mapf/utils.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace {
    struct GoalOccupancy {
        int arrivalTime;
    };

    std::string positionTimeKey(mapf::Cell* cell, int time) {
        std::ostringstream output;
        output << cell->position.x << "-" << cell->position.y << "-" << time;
        return output.str();
    }

    std::string positionKey(mapf::Cell* cell) {
        std::ostringstream output;
        output << cell->position.x << "-" << cell->position.y;
        return output.str();
    }

    std::string edgeTimeKey(mapf::Cell* from, mapf::Cell* to, int time) {
        std::ostringstream output;
        output
            << "(" << from->position.x << "," << from->position.y << ")-"
            << "(" << to->position.x << "," << to->position.y << ")-"
            << time;
        return output.str();
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

    void writeCsvRow(std::ofstream& output, const std::list<std::string>& fields) {
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
    std::unordered_map<std::string, GoalOccupancy> goalVertexLookup;
    std::unordered_map<std::string, mapf::Cell*> vertexLookup;
    std::unordered_map<std::string, mapf::Cell*> edgeLookup;

    for (const std::list<mapf::Cell*>& path : paths) {
        if (path.empty()) {
            continue;
        }

        int t = 0;
        mapf::Cell* previousCell = nullptr;

        for (mapf::Cell* cell : path) {
            std::string vertexKey = positionKey(cell);

            auto goalVertexLookupItem = goalVertexLookup.find(vertexKey);
            if (goalVertexLookupItem != goalVertexLookup.end() && goalVertexLookupItem->second.arrivalTime <= t) {
                conflicts.cellConflicts.push_back({*cell, t});
            }

            std::string vertexTimeKey = positionTimeKey(cell, t);

            if (vertexLookup.contains(vertexTimeKey)) {
                conflicts.cellConflicts.push_back({*cell, t});
            }

            vertexLookup.insert({vertexTimeKey, cell});

            if (t > 0) {
                std::string edge = edgeTimeKey(previousCell, cell, t);
                std::string invertedEdge = edgeTimeKey(cell, previousCell, t);

                if (edgeLookup.contains(invertedEdge)) {
                    conflicts.edgeConflicts.push_back({*previousCell, *cell, t});
                }

                edgeLookup.insert({edge, cell});
            }

            previousCell = cell;
            t++;
        }

        std::string endVertexKey = positionKey(previousCell);
        int arrivalTime = static_cast<int>(path.size()) - 1;
        goalVertexLookup.insert({endVertexKey, {arrivalTime}});
    }

    return conflicts;
}




bool writeResultsToCsvFile(
    const std::string& outputFilename,
    const std::list<std::string>& headers,
    const std::list<std::string>& rowValues
) {
    std::filesystem::path outputPath(outputFilename);

    if (outputPath.extension() != ".csv") {
        return false;
    }

    if (headers.size() != rowValues.size()) {
        return false;
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
    writeCsvRow(output, rowValues);

    return output.good();
}
