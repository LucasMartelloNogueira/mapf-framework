#include "mapf/solvers/priority_planning_solver.hpp"

#include "mapf/pathfinding/a_star.hpp"
#include "mapf/pathfinding/a_star_sipp.hpp"
#include "mapf/utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace mapf {

    namespace {
        double elapsedSeconds(std::chrono::steady_clock::time_point startedAt) {
            const auto finishedAt = std::chrono::steady_clock::now();
            return std::chrono::duration<double>(finishedAt - startedAt).count();
        }

        int pathCost(const std::list<Cell*>& path) {
            if (path.empty()) {
                return 0;
            }

            return static_cast<int>(path.size()) - 1;
        }

        double calculateInjustice(
            Grid& grid,
            const std::vector<const Agent*>& orderedAgents,
            const std::vector<std::list<Cell*>>& solutionPaths
        ) {
            if (orderedAgents.empty()) {
                return 0.0;
            }

            AStarSolver aStarSolver;
            std::vector<double> diffCostPaths;
            diffCostPaths.reserve(orderedAgents.size());

            for (int i = 0; i < static_cast<int>(orderedAgents.size()); i++) {
                const Agent& agent = *orderedAgents[i];
                Cell* start = grid.getCellPtr(agent.startPosition.x, agent.startPosition.y);
                Cell* goal = grid.getCellPtr(agent.goalPosition.x, agent.goalPosition.y);
                std::list<Cell*> optimalPath = aStarSolver.solve(grid, start, goal);

                if (optimalPath.empty()) {
                    throw std::runtime_error("Could not calculate unconstrained optimal path for agent.");
                }

                double diffCostPath = static_cast<double>(solutionPaths[i].size()) - static_cast<double>(optimalPath.size());
                diffCostPaths.push_back(diffCostPath);
            }

            double sum = 0.0;
            for (double diffCostPath : diffCostPaths) {
                sum += diffCostPath;
            }

            double mean = sum / static_cast<double>(diffCostPaths.size());
            double squaredDistanceSum = 0.0;

            for (double diffCostPath : diffCostPaths) {
                double distance = diffCostPath - mean;
                squaredDistanceSum += distance * distance;
            }

            return std::sqrt(squaredDistanceSum / static_cast<double>(diffCostPaths.size()));
        }
    }

    PriorityPlanningSolver::PriorityPlanningSolver(const Instance& instance) :
        instance(instance),
        result() {}

    Result PriorityPlanningSolver::solve(std::list<int> agentsOrder) {
        const auto startedAt = std::chrono::steady_clock::now();

        const std::vector<Agent>& agents = instance.getAgents();
        std::unordered_map<int, const Agent*> agentsById;

        for (const Agent& agent : agents) {
            agentsById[agent.id] = &agent;
        }

        if (agentsOrder.size() != agents.size()) {
            result = Result(false, 0, 0, 0.0, elapsedSeconds(startedAt));
            return result;
        }

        std::unordered_set<int> seenIds;
        std::vector<const Agent*> orderedAgents;
        orderedAgents.reserve(agentsOrder.size());

        for (int agentId : agentsOrder) {
            if (seenIds.contains(agentId)) {
                result = Result(false, 0, 0, 0.0, elapsedSeconds(startedAt));
                return result;
            }

            auto agentIt = agentsById.find(agentId);
            if (agentIt == agentsById.end()) {
                result = Result(false, 0, 0, 0.0, elapsedSeconds(startedAt));
                return result;
            }

            seenIds.insert(agentId);
            orderedAgents.push_back(agentIt->second);
        }

        AStarSippSolver sippSolver;
        std::vector<std::list<Cell*>> paths;
        paths.reserve(orderedAgents.size());

        Grid& grid = const_cast<Grid&>(instance.getGrid());

        for (const Agent* agent : orderedAgents) {
            Cell* start = grid.getCellPtr(agent->startPosition.x, agent->startPosition.y);
            Cell* goal = grid.getCellPtr(agent->goalPosition.x, agent->goalPosition.y);

            std::list<Cell*> path = sippSolver.solve(grid, start, goal, paths);
            if (path.empty()) {
                result = Result(false, 0, 0, 0.0, elapsedSeconds(startedAt));
                return result;
            }

            paths.push_back(std::move(path));
        }

        if (!validateSolution(paths)) {
            result = Result(false, 0, 0, 0.0, elapsedSeconds(startedAt));
            return result;
        }

        int sumOfCosts = 0;
        int makespan = 0;

        for (const std::list<Cell*>& path : paths) {
            int cost = pathCost(path);
            sumOfCosts += cost;
            makespan = std::max(makespan, cost);
        }

        double injustice = calculateInjustice(grid, orderedAgents, paths);
        result = Result(true, sumOfCosts, makespan, injustice, elapsedSeconds(startedAt));

        return result;
    }

}
