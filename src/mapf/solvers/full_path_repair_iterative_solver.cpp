#include "mapf/solvers/full_path_repair_iterative_solver.hpp"

#include "local_path_repair_solver_common.hpp"
#include "mapf/pathfinding/a_star.hpp"
#include "mapf/pathfinding/a_star_sipp.hpp"
#include "mapf/utils.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <variant>

namespace mapf {
    namespace {
        struct SelectedConflict {
            std::variant<CellTime, EdgeTime> key;
            std::vector<std::size_t> participants;
        };

        bool validCompletePath(const std::list<Cell*>& path, Cell* start, Cell* goal) {
            if (path.empty() || path.size() > static_cast<std::size_t>(SAFE_INTERVAL_INFINITY) ||
                path.front() != start || path.back() != goal) {
                return false;
            }
            Cell* previous = nullptr;
            for (Cell* cell : path) {
                if (cell == nullptr || !cell->isFree) {
                    return false;
                }
                if (previous != nullptr &&
                    std::abs(previous->position.x - cell->position.x) +
                    std::abs(previous->position.y - cell->position.y) > 1) {
                    return false;
                }
                previous = cell;
            }
            return true;
        }

        void appendParticipants(
            SelectedConflict& selected,
            const std::unordered_set<int>& participants,
            std::size_t agentCount
        ) {
            for (int index : participants) {
                if (index < 0 || static_cast<std::size_t>(index) >= agentCount) {
                    throw std::logic_error("Conflict participant is outside the agent vector.");
                }
                selected.participants.push_back(static_cast<std::size_t>(index));
            }
        }

        std::tuple<int, int, int, int, int, int, int> eventOrder(
            const SelectedConflict& selected, const std::vector<Agent>& agents
        ) {
            const int firstId = agents[selected.participants.front()].id;
            if (const auto* vertex = std::get_if<CellTime>(&selected.key)) {
                return {firstId, vertex->time, 0, vertex->cell->position.x,
                    vertex->cell->position.y, vertex->cell->position.x, vertex->cell->position.y};
            }
            const auto& edge = std::get<EdgeTime>(selected.key);
            return {firstId, edge.time, 1, edge.first->position.x, edge.first->position.y,
                edge.second->position.x, edge.second->position.y};
        }
        
        // TODO: entender essa função
        SelectedConflict selectConflict(
            const SolutionConflicts& conflicts, const std::vector<Agent>& agents
        ) {
            std::optional<SelectedConflict> selected;
            const auto consider = [&](SelectedConflict candidate) {
                auto& participants = candidate.participants;
                std::sort(participants.begin(), participants.end());
                participants.erase(std::unique(participants.begin(), participants.end()), participants.end());
                if (participants.size() < 2) {
                    throw std::logic_error("A conflict must have at least two distinct participants.");
                }
                std::sort(participants.begin(), participants.end(), [&](std::size_t a, std::size_t b) {
                    return agents[a].id < agents[b].id;
                });
                if (!selected || eventOrder(candidate, agents) < eventOrder(*selected, agents)) {
                    selected = std::move(candidate);
                }
            };
            for (const auto& [key, event] : conflicts.vertexEvents) {
                SelectedConflict candidate{key, {}};
                appendParticipants(candidate, event.participants, agents.size());
                consider(std::move(candidate));
            }
            for (const auto& [key, event] : conflicts.edgeEvents) {
                if (!event.active()) {
                    throw std::logic_error("Inactive edge event in conflict snapshot.");
                }
                SelectedConflict candidate{key, {}};
                appendParticipants(candidate, event.forward, agents.size());
                appendParticipants(candidate, event.reverse, agents.size());
                consider(std::move(candidate));
            }
            if (!selected) {
                throw std::logic_error("No event selected from the conflict snapshot.");
            }
            return std::move(*selected);
        }

        bool hasParticipantConflict(
            const SolutionConflicts& conflicts, const std::vector<std::size_t>& participants
        ) {
            // These indexes originated in getCollision's checked int participant sets.
            const std::unordered_set<int> members(participants.begin(), participants.end());
            const auto intersects = [&](const std::unordered_set<int>& owners) {
                return std::any_of(owners.begin(), owners.end(), [&](int index) {
                    return members.contains(index);
                });
            };
            for (const auto& [key, event] : conflicts.vertexEvents) {
                if (intersects(event.participants)) {
                    return true;
                }
            }
            for (const auto& [key, event] : conflicts.edgeEvents) {
                if (intersects(event.forward) || intersects(event.reverse)) {
                    return true;
                }
            }
            return false;
        }

        std::size_t eventCount(const SolutionConflicts& conflicts) {
            return conflicts.vertexEvents.size() + conflicts.edgeEvents.size();
        }
    }

    FullPathRepairIterativeSolver::FullPathRepairIterativeSolver(const Instance& instance) :
        instance(instance) {}

    LocalPathRepairResult FullPathRepairIterativeSolver::solve() {
        const auto startedAt = std::chrono::steady_clock::now();
        const auto& agents = instance.getAgents();
        Grid& grid = const_cast<Grid&>(instance.getGrid());
        LocalPathRepairResult result;
        result.initialPaths.resize(agents.size());
        for (std::size_t i = 0; i < agents.size(); ++i) {
            AStarSolver astar;
            result.initialPaths[i] = astar.solve(grid,
                grid.getCellPtr(agents[i].startPosition.x, agents[i].startPosition.y),
                grid.getCellPtr(agents[i].goalPosition.x, agents[i].goalPosition.y));
        }

        // TODO: retirar essa verificacao do caminho
        //       se astar.solve(...) retornar caminho vazio ou nullptr, encerrar programa ali
        bool allInitialPathsExist = true;
        for (std::size_t i = 0; i < agents.size(); ++i) {
            if (result.initialPaths[i].empty()) {
                allInitialPathsExist = false;
            } else if (!validCompletePath(result.initialPaths[i],
                grid.getCellPtr(agents[i].startPosition.x, agents[i].startPosition.y),
                grid.getCellPtr(agents[i].goalPosition.x, agents[i].goalPosition.y))) {
                throw std::invalid_argument("Malformed initial path or unsupported SIPP time.");
            }
        }
        result.paths = result.initialPaths;
        result.initialConflicts = getCollision(result.initialPaths);
        result.remainingConflicts = result.initialConflicts;
        result.reservations = local_path_repair_detail::buildReservationState(grid, agents, result.paths);
        if (!allInitialPathsExist) {
            local_path_repair_detail::updateResultMetrics(result, agents, false, startedAt);
            return result;
        }

        AStarSippSolver sipp;
        while (!result.remainingConflicts.empty()) {
            const auto selected = selectConflict(result.remainingConflicts, agents);
            auto candidatePaths = result.paths;
            auto candidateReservations = result.reservations;

            // Exclude the complete group before searching for ANY replacement.
            for (std::size_t index : selected.participants) {
                local_path_repair_detail::repairSafeIntervalTable(
                    candidateReservations, result.paths[index], agents[index].id);
            }
            for (std::size_t index : selected.participants) {
                Cell* start = grid.getCellPtr(agents[index].startPosition.x, agents[index].startPosition.y);
                Cell* goal = grid.getCellPtr(agents[index].goalPosition.x, agents[index].goalPosition.y);
                auto replacement = sipp.solve(grid, start, goal, candidateReservations.safeIntervalTable,
                    0, AStarSippSolver::GoalOccupation::Permanent);
                if (!validCompletePath(replacement, start, goal)) {
                    local_path_repair_detail::updateResultMetrics(result, agents, false, startedAt);
                    return result;
                }
                local_path_repair_detail::updateReservationState(
                    candidateReservations, replacement, agents[index].id);
                candidatePaths[index] = std::move(replacement);
            }
            
            // TODO: não recalcular todos os conflitos, apenas retirar o conflito que excluido da lista de conflitos (result.remainingConflicts)
            auto candidateConflicts = getCollision(candidatePaths);
            if (hasParticipantConflict(candidateConflicts, selected.participants) ||
                eventCount(candidateConflicts) >= eventCount(result.remainingConflicts)) {
                local_path_repair_detail::updateResultMetrics(result, agents, false, startedAt);
                return result;
            }

            // Publish only a complete group, after all allocating work and validation.
            static_assert(std::is_nothrow_move_assignable_v<decltype(result.paths)>);
            static_assert(std::is_nothrow_move_assignable_v<SolutionConflicts>);
            static_assert(std::is_nothrow_move_assignable_v<PathReservationState>);
            result.paths = std::move(candidatePaths);
            result.remainingConflicts = std::move(candidateConflicts);
            result.reservations = std::move(candidateReservations);
        }
        local_path_repair_detail::updateResultMetrics(result, agents, true, startedAt);
        return result;
    }
}
