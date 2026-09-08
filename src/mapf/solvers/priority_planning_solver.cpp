#include "mapf/solvers/priority_planning_solver.hpp"

#include "mapf/pathfinding/a_star.hpp"
#include "mapf/pathfinding/a_star_sipp.hpp"
#include "mapf/utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mapf {

    namespace {
        double elapsedSeconds(std::chrono::steady_clock::time_point startedAt) {
            return std::chrono::duration<double>(
                std::chrono::steady_clock::now() - startedAt
            ).count();
        }

        int pathCost(const std::list<Cell*>& path) {
            return path.empty() ? 0 : static_cast<int>(path.size()) - 1;
        }

        double calculateInjustice(
            const std::vector<std::list<Cell*>>& initialPaths,
            const std::vector<std::list<Cell*>>& solutionPaths
        ) {
            std::vector<double> differences;
            differences.reserve(std::min(initialPaths.size(), solutionPaths.size()));

            double sum = 0.0;
            const std::size_t pathCount = std::min(initialPaths.size(), solutionPaths.size());
            for (std::size_t i = 0; i < pathCount; i++) {
                if (initialPaths[i].empty() || solutionPaths[i].empty()) {
                    continue;
                }

                const double difference = static_cast<double>(
                    pathCost(solutionPaths[i]) - pathCost(initialPaths[i])
                );
                differences.push_back(difference);
                sum += difference;
            }

            if (differences.empty()) {
                return 0.0;
            }

            const double mean = sum / static_cast<double>(differences.size());
            double squaredDistanceSum = 0.0;
            for (double difference : differences) {
                const double distance = difference - mean;
                squaredDistanceSum += distance * distance;
            }

            return std::sqrt(
                squaredDistanceSum / static_cast<double>(differences.size())
            );
        }

        Result summarize(
            bool requestedSuccess,
            const std::vector<std::list<Cell*>>& initialPaths,
            const std::vector<std::list<Cell*>>& paths,
            std::chrono::steady_clock::time_point startedAt
        ) {
            int sumOfCosts = 0;
            int makespan = 0;
            bool allPathsExist = initialPaths.size() == paths.size();

            for (const std::list<Cell*>& path : paths) {
                if (path.empty()) {
                    allPathsExist = false;
                    continue;
                }

                const int cost = pathCost(path);
                sumOfCosts += cost;
                makespan = std::max(makespan, cost);
            }

            return Result(
                requestedSuccess && allPathsExist && validateSolution(paths),
                sumOfCosts,
                makespan,
                calculateInjustice(initialPaths, paths),
                elapsedSeconds(startedAt)
            );
        }
    }

    PriorityPlanningSolver::PriorityPlanningSolver(const Instance& instance) :
        instance(instance),
        result(),
        initialPaths(),
        paths() {}

    Result PriorityPlanningSolver::solve(std::list<int> agentsOrder) {
        const auto startedAt = std::chrono::steady_clock::now();
        const std::vector<Agent>& agents = instance.getAgents();
        initialPaths.assign(agents.size(), {});
        paths.assign(agents.size(), {});

        if (agentsOrder.size() != agents.size()) {
            result = summarize(false, initialPaths, paths, startedAt);
            return result;
        }

        std::unordered_map<int, std::size_t> indexesById;
        indexesById.reserve(agents.size());
        for (std::size_t i = 0; i < agents.size(); i++) {
            indexesById.emplace(agents[i].id, i);
        }

        std::unordered_set<int> seenIds;
        std::vector<std::size_t> orderedIndexes;
        orderedIndexes.reserve(agents.size());
        for (int agentId : agentsOrder) {
            const auto agentIt = indexesById.find(agentId);
            if (!seenIds.insert(agentId).second || agentIt == indexesById.end()) {
                result = summarize(false, initialPaths, paths, startedAt);
                return result;
            }

            orderedIndexes.push_back(agentIt->second);
        }

        Grid& grid = const_cast<Grid&>(instance.getGrid());
        bool allInitialPathsExist = true;
        for (std::size_t i = 0; i < agents.size(); i++) {
            AStarSolver aStarSolver;
            Cell* start = grid.getCellPtr(
                agents[i].startPosition.x,
                agents[i].startPosition.y
            );
            Cell* goal = grid.getCellPtr(
                agents[i].goalPosition.x,
                agents[i].goalPosition.y
            );
            initialPaths[i] = aStarSolver.solve(grid, start, goal);
            if (initialPaths[i].empty()) {
                allInitialPathsExist = false;
            }
        }

        if (!allInitialPathsExist) {
            result = summarize(false, initialPaths, paths, startedAt);
            return result;
        }

        AStarSippSolver sippSolver;
        std::vector<std::list<Cell*>> reservedPaths;
        reservedPaths.reserve(agents.size());

        for (std::size_t agentIndex : orderedIndexes) {
            const Agent& agent = agents[agentIndex];
            Cell* start = grid.getCellPtr(
                agent.startPosition.x,
                agent.startPosition.y
            );
            Cell* goal = grid.getCellPtr(
                agent.goalPosition.x,
                agent.goalPosition.y
            );

            std::list<Cell*> path = sippSolver.solve(
                grid,
                start,
                goal,
                reservedPaths
            );
            if (path.empty()) {
                result = summarize(false, initialPaths, paths, startedAt);
                return result;
            }

            paths[agentIndex] = path;
            reservedPaths.push_back(std::move(path));
        }

        result = summarize(true, initialPaths, paths, startedAt);
        return result;
    }

    const std::vector<std::list<Cell*>>& PriorityPlanningSolver::getInitialPaths() const {
        return initialPaths;
    }

    const std::vector<std::list<Cell*>>& PriorityPlanningSolver::getPaths() const {
        return paths;
    }

}
