#pragma once

#include <list>
#include <vector>
#include <string>

#include "cell.hpp"
#include "agent.hpp"
#include "grid.hpp"
#include "result.hpp"

namespace mapf {
    
    class Instance {
        private:
            int numRows;
            int numCols;
            std::vector<Agent> agents;
            Grid grid;
            std::string mapName;
            std::string instanceName;

        public:
            // manual instance
            Instance(std::vector<std::vector<int>>* free, int rows, int cols, std::vector<Agent> agents);

            // instance from file
            Instance(std::string mapFilename, std::string scenarioFilename, int numAgents);

            Grid& getGrid();
            const Grid& getGrid() const;
            const std::vector<Agent>& getAgents() const;
            int getNumRows() const;
            int getNumCols() const;
            int getNumAgents() const;
            const std::string& getMapName() const;
            const std::string& getInstanceName() const;

    };
}
