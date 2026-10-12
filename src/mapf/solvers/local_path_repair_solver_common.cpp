#include "local_path_repair_solver_common.hpp"
#include "solver_utils.hpp"

#include "mapf/pathfinding/a_star_sipp.hpp"
#include "mapf/utils.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <variant>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mapf {

    namespace {
        struct SelectedConflict {
            std::size_t pathIndex;
            std::variant<CellConflict, EdgeConflict> conflict;
        };

        bool isValidStep(Cell* from, Cell* to) {
            if (from == nullptr || to == nullptr) {
                return false;
            }

            if (from == to) {
                return true;
            }

            const int distance =
                std::abs(from->position.x - to->position.x) +
                std::abs(from->position.y - to->position.y);
            return distance == 1;
        }

        bool structurallyValid(const std::list<Cell*>& path, Cell* start, Cell* goal) {
            if (path.empty() || path.size() > static_cast<std::size_t>(SAFE_INTERVAL_INFINITY) ||
                path.front() != start || path.back() != goal) {
                return false;
            }
            Cell* previous = nullptr;
            for (Cell* cell : path) {
                if (cell == nullptr || !cell->isFree ||
                    (previous != nullptr && !isValidStep(previous, cell))) {
                    return false;
                }
                previous = cell;
            }
            return true;
        }

        std::string configurationFingerprint(
            const std::vector<std::list<Cell*>>& paths,
            std::size_t replacedIndex,
            const std::list<Cell*>& replacement
        ) {
            std::ostringstream output;
            for (std::size_t i = 0; i < paths.size(); ++i) {
                const auto& path = i == replacedIndex ? replacement : paths[i];
                output << path.size() << ':';
                for (Cell* cell : path) {
                    output << cell->position.x << ',' << cell->position.y << ';';
                }
                output << '|';
            }
            return output.str();
        }

        std::optional<SelectedConflict> selectByAgent(const SolutionConflicts& conflicts) {
            for (std::size_t i = 0; i < conflicts.byAgent.size(); ++i) {
                if (!conflicts.byAgent[i].empty()) {
                    return SelectedConflict{i, conflicts.byAgent[i].front()};
                }
            }
            return std::nullopt;
        }

        std::optional<SelectedConflict> selectByTime(const SolutionConflicts& conflicts) {
            std::optional<SelectedConflict> earliest;
            for (std::size_t i = 0; i < conflicts.byAgent.size(); ++i) {
                const auto& records = conflicts.byAgent[i];
                // Iteration order breaks identical-record ties by path index.
                if (!records.empty() && (!earliest ||
                    conflictRecordComesBefore(records.front(), earliest->conflict))) {
                    earliest = SelectedConflict{i, records.front()};
                }
            }
            return earliest;
        }

        std::vector<std::size_t> adaptiveAnchors(std::size_t initialAnchor) {
            std::vector<std::size_t> anchors;
            std::unordered_set<std::size_t> seen;
            std::size_t anchor = initialAnchor;

            while (seen.insert(anchor).second) {
                anchors.push_back(anchor);
                if (anchor == 0) {
                    break;
                }

                anchor /= 2;
            }

            return anchors;
        }
    }

    void local_path_repair_detail::validateLocalRepairStrategy(LocalRepairStrategy strategy) {
        switch (strategy) {
        case LocalRepairStrategy::RESOLVE_BY_AGENT:
        case LocalRepairStrategy::RESOLVE_BY_TIME:
            return;
        default:
            throw std::invalid_argument("Unknown local repair strategy.");
        }
    }

    LocalPathRepairResult local_path_repair_detail::repairInitialPaths(
        const Instance& instance,
        std::vector<std::list<Cell*>> initialPaths,
        std::chrono::steady_clock::time_point startedAt,
        LocalRepairStrategy localRepairStrategy
    ) {
        validateLocalRepairStrategy(localRepairStrategy);
        const std::vector<Agent>& agents = instance.getAgents();
        if (initialPaths.size() != agents.size()) {
            throw std::invalid_argument("Initial paths must be aligned with instance agents.");
        }
        Grid& grid = const_cast<Grid&>(instance.getGrid());

        LocalPathRepairResult result;
        result.initialPaths = std::move(initialPaths);
        result.paths = result.initialPaths;
        result.initialConflicts = getCollision(result.initialPaths);
        result.remainingConflicts = result.initialConflicts;
        result.reservations = solver_utils::buildReservationState(grid, agents, result.paths);

        AStarSippSolver sipp;
        std::unordered_set<std::string> visitedConfigurations;
        visitedConfigurations.insert(configurationFingerprint(result.paths, result.paths.size(), {}));

        while (true) {
            std::optional<SelectedConflict> selected;
            switch (localRepairStrategy) {
            case LocalRepairStrategy::RESOLVE_BY_AGENT:
                selected = selectByAgent(result.remainingConflicts);
                break;
            case LocalRepairStrategy::RESOLVE_BY_TIME:
                selected = selectByTime(result.remainingConflicts);
                break;
            default:
                throw std::invalid_argument("Unknown local repair strategy.");
            }
            if (!selected) {
                break;
            }

            const std::size_t activeIndex = selected->pathIndex;
            const std::vector<Cell*> oldPath(result.paths[activeIndex].begin(), result.paths[activeIndex].end());
            solver_utils::ReservationTransaction repair(result.reservations);
            repair.removePath(result.paths[activeIndex], agents[activeIndex].id);

            std::list<Cell*> newPath;
            std::string nextConfiguration;
            bool repairSuccess = false;
            int repairStartIndex = 0;
            const int time = conflictTime(selected->conflict);
            if (time > 0 && static_cast<std::size_t>(time) < oldPath.size()) {
                const std::size_t reconnectIndex = static_cast<std::size_t>(time) +
                    (std::holds_alternative<CellConflict>(selected->conflict) ? 1 : 0);
                if (reconnectIndex < oldPath.size()) {
                    const auto goalOccupation = reconnectIndex + 1 == oldPath.size()
                        ? AStarSippSolver::GoalOccupation::Permanent
                        : AStarSippSolver::GoalOccupation::Transient;
                    for (std::size_t anchor : adaptiveAnchors(static_cast<std::size_t>(time - 1))) {
                        const std::list<Cell*> bridge = sipp.solve(grid, oldPath[anchor],
                            oldPath[reconnectIndex], result.reservations.safeIntervalTable,
                            static_cast<int>(anchor), goalOccupation);
                        if (!structurallyValid(bridge, oldPath[anchor], oldPath[reconnectIndex])) {
                            continue;
                        }
                        const std::size_t tailSize = oldPath.size() - reconnectIndex - 1;
                        const std::size_t limit = static_cast<std::size_t>(SAFE_INTERVAL_INFINITY);
                        if (bridge.size() > limit - anchor || tailSize > limit - anchor - bridge.size()) {
                            continue;
                        }

                        // Preserve the prefix and suffix without duplicating the bridge endpoints.
                        newPath.assign(oldPath.begin(), oldPath.begin() + static_cast<std::ptrdiff_t>(anchor));
                        newPath.insert(newPath.end(), bridge.begin(), bridge.end());
                        newPath.insert(newPath.end(),
                            oldPath.begin() + static_cast<std::ptrdiff_t>(reconnectIndex + 1), oldPath.end());
                        if (!structurallyValid(newPath, oldPath.front(), oldPath.back())) {
                            continue;
                        }
                        nextConfiguration = configurationFingerprint(result.paths, activeIndex, newPath);
                        if (visitedConfigurations.contains(nextConfiguration)) {
                            continue;
                        }
                        repairStartIndex = static_cast<int>(anchor);
                        repairSuccess = true;
                        break;
                    }
                }
            }

            if (!repairSuccess && !oldPath.empty()) {
                newPath = sipp.solve(grid, oldPath.front(), oldPath.back(),
                    result.reservations.safeIntervalTable, 0, AStarSippSolver::GoalOccupation::Permanent);
                if (structurallyValid(newPath, oldPath.front(), oldPath.back())) {
                    nextConfiguration = configurationFingerprint(result.paths, activeIndex, newPath);
                    repairSuccess = !visitedConfigurations.contains(nextConfiguration);
                    repairStartIndex = 0;
                }
            }
            if (!repairSuccess) {
                // Restore before returning: result may be moved before local
                // destructors run when named return value optimization is absent.
                repair.rollback();
                solver_utils::updateResultMetrics(result, agents, false, startedAt);
                return result;
            }

            repair.insertPath(newPath, agents[activeIndex].id);

            SolutionConflicts updatedConflicts = UpdateSolutionConflictsV2(
                newPath, result.reservations, result.remainingConflicts,
                static_cast<int>(activeIndex), repairStartIndex);
            visitedConfigurations.insert(std::move(nextConfiguration));
            // All allocating work finished. Publishing cannot leave paths and
            // reservations inconsistent if an earlier operation throws.
            static_assert(std::is_nothrow_move_assignable_v<decltype(newPath)>);
            static_assert(std::is_nothrow_move_assignable_v<SolutionConflicts>);
            result.paths[activeIndex] = std::move(newPath);
            result.remainingConflicts = std::move(updatedConflicts);
            repair.commit();
        }

        solver_utils::updateResultMetrics(result, agents, true, startedAt);
        return result;
    }

}
