#include "mapf/core/instance.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace mapf {

    namespace {
        void require(bool condition, const std::string& message) {
            if (!condition) {
                throw std::runtime_error(message);
            }
        }

        std::string readHeaderValue(std::istream& input, const std::string& expectedKey) {
            std::string key;
            std::string value;
            input >> key >> value;

            require(input.good() || input.eof(), "Failed to read map header.");
            require(key == expectedKey, "Expected map header key '" + expectedKey + "'.");

            return value;
        }

        int readHeaderInt(std::istream& input, const std::string& expectedKey) {
            std::string value = readHeaderValue(input, expectedKey);
            std::size_t processed = 0;
            int parsed = std::stoi(value, &processed);

            require(processed == value.size(), "Invalid integer value for map header '" + expectedKey + "'.");
            require(parsed > 0, "Map header '" + expectedKey + "' must be positive.");

            return parsed;
        }

        bool isFreeCell(const std::vector<std::vector<int>>& freeCells, int x, int y) {
            if (y < 0 || y >= static_cast<int>(freeCells.size())) {
                return false;
            }

            if (x < 0 || x >= static_cast<int>(freeCells[y].size())) {
                return false;
            }

            return freeCells[y][x] == 1;
        }
    }

    Instance::Instance(
        std::vector<std::vector<int>>* free,
        int rows,
        int cols,
        std::vector<Agent> agents
    ) :
        numRows(rows),
        numCols(cols),
        agents(std::move(agents)),
        grid(free, rows, cols),
        mapName("-"),
        instanceName("-")
    {
        std::unordered_set<int> ids;

        for (const Agent& agent : this->agents) {
            require(ids.insert(agent.id).second, "Agent IDs must be unique.");

            Cell* start = grid.getCellPtr(agent.startPosition.x, agent.startPosition.y);
            Cell* goal = grid.getCellPtr(agent.goalPosition.x, agent.goalPosition.y);

            require(start != nullptr && start->isFree, "Agent start position must be a free cell.");
            require(goal != nullptr && goal->isFree, "Agent goal position must be a free cell.");
        }
    }

    Instance::Instance(std::string mapFilename, std::string scenarioFilename, int numAgents) :
        numRows(0),
        numCols(0),
        agents(),
        grid(1, 1),
        mapName(std::filesystem::path(mapFilename).filename().string()),
        instanceName(std::filesystem::path(scenarioFilename).filename().string())
    {
        require(numAgents >= 0, "Number of agents must be non-negative.");

        std::ifstream mapInput(mapFilename);
        require(mapInput.is_open(), "Could not open map file '" + mapFilename + "'.");

        const std::string mapType = readHeaderValue(mapInput, "type");
        (void) mapType;

        int height = readHeaderInt(mapInput, "height");
        int width = readHeaderInt(mapInput, "width");

        std::string mapKey;
        mapInput >> mapKey;
        require(mapKey == "map", "Expected map data marker.");

        std::string mapLine;
        std::getline(mapInput, mapLine);

        std::vector<std::vector<int>> freeCells(height, std::vector<int>(width, 0));

        for (int y = 0; y < height; y++) {
            require(static_cast<bool>(std::getline(mapInput, mapLine)), "Map file ended before all rows were read.");
            require(static_cast<int>(mapLine.size()) == width, "Map row width does not match header width.");

            for (int x = 0; x < width; x++) {
                freeCells[y][x] = mapLine[x] == '.' ? 1 : 0;
            }
        }

        std::ifstream scenarioInput(scenarioFilename);
        require(scenarioInput.is_open(), "Could not open scenario file '" + scenarioFilename + "'.");

        std::string scenarioHeader;
        std::getline(scenarioInput, scenarioHeader);
        require(scenarioHeader.rfind("version", 0) == 0, "Expected scenario version header.");

        std::vector<Agent> parsedAgents;
        parsedAgents.reserve(numAgents);

        std::string scenarioLine;
        int agentId = 0;

        while (static_cast<int>(parsedAgents.size()) < numAgents && std::getline(scenarioInput, scenarioLine)) {
            if (scenarioLine.empty()) {
                continue;
            }

            std::istringstream lineInput(scenarioLine);
            int bucket = 0;
            std::string scenarioMapName;
            int scenarioWidth = 0;
            int scenarioHeight = 0;
            int startX = 0;
            int startY = 0;
            int goalX = 0;
            int goalY = 0;
            double optimalLength = 0.0;

            lineInput
                >> bucket
                >> scenarioMapName
                >> scenarioWidth
                >> scenarioHeight
                >> startX
                >> startY
                >> goalX
                >> goalY
                >> optimalLength;

            require(!lineInput.fail(), "Invalid scenario row.");
            require(scenarioMapName == mapName, "Scenario map name does not match map filename.");
            require(scenarioWidth == width && scenarioHeight == height, "Scenario dimensions do not match map dimensions.");
            require(isFreeCell(freeCells, startX, startY), "Scenario start position must be a free map cell.");
            require(isFreeCell(freeCells, goalX, goalY), "Scenario goal position must be a free map cell.");

            Position startPosition {
                .x = startX,
                .y = startY
            };

            Position goalPosition {
                .x = goalX,
                .y = goalY
            };

            parsedAgents.push_back({
                .id = agentId,
                .currentPosition = startPosition,
                .startPosition = startPosition,
                .goalPosition = goalPosition
            });

            agentId++;
        }

        require(static_cast<int>(parsedAgents.size()) == numAgents, "Scenario file does not contain enough agent rows.");

        numRows = height;
        numCols = width;
        agents = std::move(parsedAgents);
        grid = Grid(&freeCells, height, width);
    }

    Grid& Instance::getGrid() {
        return grid;
    }

    const Grid& Instance::getGrid() const {
        return grid;
    }

    const std::vector<Agent>& Instance::getAgents() const {
        return agents;
    }

    int Instance::getNumRows() const {
        return numRows;
    }

    int Instance::getNumCols() const {
        return numCols;
    }

    int Instance::getNumAgents() const {
        return static_cast<int>(agents.size());
    }

    const std::string& Instance::getMapName() const {
        return mapName;
    }

    const std::string& Instance::getInstanceName() const {
        return instanceName;
    }

}
