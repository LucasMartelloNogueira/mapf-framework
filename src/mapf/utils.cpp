#include "mapf/utils.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace {
    std::string positionTimeKey(mapf::Cell* cell, int time) {
        std::ostringstream output;
        output << cell->position.x << "-" << cell->position.y << "-" << time;
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
    std::unordered_set<std::string> verticeColisions;
    std::unordered_set<std::string> edgeColisions;

    int i = 0;
    for (const std::list<mapf::Cell*>& path : paths) {

        int t = 0;
        mapf::Cell* previousCell = nullptr;

        for (mapf::Cell* cell : path) {
            
            std::string verticeAtTime = positionTimeKey(cell, t);

            if (verticeColisions.contains(verticeAtTime)) {

                std::printf("ja contem o vertice no timesetp %s, i = %d\n", verticeAtTime.c_str(), i);
                return false;
            }

            verticeColisions.insert(verticeAtTime);

            // checking edge conflict
            if (t > 0) {
                std::string edge = edgeTimeKey(cell, previousCell, t);
                std::string invertedEdge = edgeTimeKey(previousCell, cell, t);

                if (edgeColisions.contains(edge) || edgeColisions.contains(invertedEdge)) {
                    std::printf("aresta %s já foi usada no tempo %d\n", edge.c_str(), t);
                    return false;
                }

                edgeColisions.insert(edge);
            }
            previousCell = cell;
            t++;
        }
        i++;
    }

    return true;
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
