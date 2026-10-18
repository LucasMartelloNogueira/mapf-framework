#include "mapf/solvers/local_path_repair_iterative_solver.hpp"

#include "local_path_repair_solver_common.hpp"

#include "mapf/pathfinding/a_star.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>
#include <vector>

namespace mapf {

    LocalPathRepairIterativeSolver::LocalPathRepairIterativeSolver(
        const Instance& instance,
        LocalRepairStrategy localRepairStrategy,
        int maxNewConflicts
    ) :
        instance(instance),
        localRepairStrategy(localRepairStrategy),
        maxNewConflicts(maxNewConflicts) {
        local_path_repair_detail::validateLocalRepairStrategy(localRepairStrategy);
        if (maxNewConflicts < 0) {
            throw std::invalid_argument("The maximum number of new conflicts must be nonnegative.");
        }
    }

    LocalPathRepairResult LocalPathRepairIterativeSolver::solve() {
        const auto startedAt = std::chrono::steady_clock::now();
        const std::vector<Agent>& agents = instance.getAgents();
        std::vector<std::list<Cell*>> initialPaths(agents.size());
        Grid& grid = const_cast<Grid&>(instance.getGrid());

        for (std::size_t i = 0; i < agents.size(); i++) {
            const Agent& agent = agents[i];
            AStarSolver solver;
            Cell* start = grid.getCellPtr(
                agent.startPosition.x,
                agent.startPosition.y
            );
            Cell* goal = grid.getCellPtr(
                agent.goalPosition.x,
                agent.goalPosition.y
            );
            auto path = solver.solve(grid, start, goal);
            if (path.empty()) {
                return local_path_repair_detail::repairInitialPaths(
                    instance,
                    std::move(initialPaths),
                    startedAt,
                    localRepairStrategy,
                    maxNewConflicts
                );
            }
            initialPaths[i] = std::move(path);
        }

        return local_path_repair_detail::repairInitialPaths(
            instance,
            std::move(initialPaths),
            startedAt,
            localRepairStrategy,
            maxNewConflicts
        );
    }

}
