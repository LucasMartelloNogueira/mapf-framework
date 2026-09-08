#include "mapf/solvers/local_path_repair_iterative_solver.hpp"

#include "local_path_repair_solver_common.hpp"

#include "mapf/pathfinding/a_star.hpp"

#include <chrono>
#include <utility>
#include <vector>

namespace mapf {

    LocalPathRepairIterativeSolver::LocalPathRepairIterativeSolver(
        const Instance& instance,
        bool continueIfFailed
    ) :
        instance(instance),
        continueIfFailed(continueIfFailed) {}

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
            initialPaths[i] = solver.solve(grid, start, goal);
        }

        return local_path_repair_detail::repairInitialPaths(
            instance,
            std::move(initialPaths),
            continueIfFailed,
            startedAt
        );
    }

}
