#include "local_path_repair_solver_common.hpp"
#include "solver_utils.hpp"

#include "mapf/pathfinding/a_star_sipp.hpp"
#include "mapf/utils.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

namespace mapf {

    namespace {
        struct SelectedConflict {
            std::size_t pathIndex;
            std::variant<CellConflict, EdgeConflict> conflict;
        };

        struct Hash128 {
            std::uint64_t low = 0;
            std::uint64_t high = 0;

            bool operator==(const Hash128&) const = default;

            Hash128& operator^=(const Hash128& other) {
                low ^= other.low;
                high ^= other.high;
                return *this;
            }
        };

        Hash128 operator^(Hash128 first, const Hash128& second) {
            return first ^= second;
        }

        std::uint64_t avalanche64(std::uint64_t value) {
            value ^= value >> 30;
            value *= UINT64_C(0xbf58476d1ce4e5b9);
            value ^= value >> 27;
            value *= UINT64_C(0x94d049bb133111eb);
            value ^= value >> 31;
            return value;
        }

        Hash128 mix128(
            std::uint64_t first,
            std::uint64_t second,
            std::uint64_t domain
        ) {
            const std::uint64_t low = avalanche64(
                first ^ std::rotl(second, 17) ^ domain
            );
            const std::uint64_t high = avalanche64(
                second ^ std::rotl(first, 41) ^ ~domain ^ low
            );
            return {low, high};
        }

        Hash128 hashPath(const std::list<Cell*>& path, int gridWidth) {
            constexpr std::uint64_t PATH_STEP_DOMAIN = UINT64_C(0x6f52f24f9b3d4a71);
            constexpr std::uint64_t PATH_LENGTH_DOMAIN = UINT64_C(0xb8a7d6c519e2430f);

            Hash128 hash;
            std::uint64_t time = 0;
            for (const Cell* cell : path) {
                const std::uint64_t cellId =
                    static_cast<std::uint64_t>(cell->position.y) *
                    static_cast<std::uint64_t>(gridWidth) +
                    static_cast<std::uint64_t>(cell->position.x);
                hash ^= mix128(time, cellId, PATH_STEP_DOMAIN);
                ++time;
            }
            hash ^= mix128(
                static_cast<std::uint64_t>(path.size()),
                0,
                PATH_LENGTH_DOMAIN
            );
            return hash;
        }

        Hash128 agentContribution(std::size_t agentIndex, const Hash128& pathHash) {
            constexpr std::uint64_t AGENT_DOMAIN = UINT64_C(0xd37c8e9a41b5620f);
            const Hash128 indexedLow = mix128(
                static_cast<std::uint64_t>(agentIndex),
                pathHash.low,
                AGENT_DOMAIN
            );
            const Hash128 indexedHigh = mix128(
                static_cast<std::uint64_t>(agentIndex),
                pathHash.high,
                ~AGENT_DOMAIN
            );
            return indexedLow ^ indexedHigh;
        }

        struct Hash128Hasher {
            std::size_t operator()(const Hash128& hash) const {
                return static_cast<std::size_t>(avalanche64(
                    hash.low ^ std::rotl(hash.high, 29)
                ));
            }
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
        LocalRepairStrategy localRepairStrategy,
        int maxNewConflicts
    ) {
        validateLocalRepairStrategy(localRepairStrategy);
        if (maxNewConflicts < 0) {
            throw std::invalid_argument("The maximum number of new conflicts must be nonnegative.");
        }
        const std::vector<Agent>& agents = instance.getAgents();
        if (initialPaths.size() != agents.size()) {
            throw std::invalid_argument("Initial paths must be aligned with instance agents.");
        }
        Grid& grid = const_cast<Grid&>(instance.getGrid());

        LocalPathRepairResult result;
        result.agentAnchorsIndex.assign(agents.size(), AGENT_PATH_UNCHANGED);
        result.initialPaths = std::move(initialPaths);
        result.paths = result.initialPaths;
        result.initialConflicts = getCollision(result.initialPaths);
        result.remainingConflicts = result.initialConflicts;
        result.reservations = solver_utils::buildReservationState(grid, agents, result.paths);

        std::vector<Hash128> pathHashes;
        pathHashes.reserve(result.paths.size());
        Hash128 configHash;
        for (std::size_t i = 0; i < result.paths.size(); ++i) {
            pathHashes.push_back(hashPath(result.paths[i], instance.getNumCols()));
            configHash ^= agentContribution(i, pathHashes.back());
        }
        std::unordered_set<Hash128, Hash128Hasher> visitedConfigurations;
        visitedConfigurations.insert(configHash);
        std::size_t sameConfigCounter = 0;

        AStarSippSolver sipp;

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
            const int activeAgentId = agents[activeIndex].id;
            const std::vector<Cell*> oldPath(result.paths[activeIndex].begin(), result.paths[activeIndex].end());
            solver_utils::ReservationTransaction repair(result.reservations);
            repair.removePath(result.paths[activeIndex], activeAgentId);

            std::list<Cell*> newPath;
            bool repairSuccess = false;
            int repairStartIndex = 0;
            int repairOutcome = AGENT_REPAIR_FAILED;
            Hash128 acceptedPathHash;
            Hash128 acceptedConfigHash;
            const int time = conflictTime(selected->conflict);
            if (time > 0 && static_cast<std::size_t>(time) < oldPath.size()) {
                const std::size_t reconnectIndex = static_cast<std::size_t>(time) +
                    (std::holds_alternative<CellConflict>(selected->conflict) ? 1 : 0);
                if (reconnectIndex < oldPath.size()) {
                    const auto goalOccupation = reconnectIndex + 1 == oldPath.size()
                        ? AStarSippSolver::GoalOccupation::Permanent
                        : AStarSippSolver::GoalOccupation::Transient;
                    int anchorIndex = 0;
                    for (std::size_t anchor : adaptiveAnchors(static_cast<std::size_t>(time - 1))) {
                        // Advance even when a rejected candidate uses continue.
                        const int currentAnchorIndex = anchorIndex++;
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
                        // The bridge is safe, but its arrival time can shift the
                        // preserved suffix into another agent's reservations.
                        const std::list<Cell*> pathFromAnchor(
                            std::next(newPath.cbegin(), static_cast<std::ptrdiff_t>(anchor)),
                            newPath.cend());
                        if (!solver_utils::canAcceptNewPath(
                            result.reservations.vertex_agents,
                            result.reservations.goal_reservations,
                            pathFromAnchor, activeAgentId, static_cast<int>(anchor),
                            maxNewConflicts)) {
                            continue;
                        }
                        const Hash128 newPathHash = hashPath(newPath, instance.getNumCols());
                        const Hash128 candidateHash = configHash ^
                            agentContribution(activeIndex, pathHashes[activeIndex]) ^
                            agentContribution(activeIndex, newPathHash);
                        if (visitedConfigurations.contains(candidateHash)) {
                            ++sameConfigCounter;
                            continue;
                        }
                        repairStartIndex = static_cast<int>(anchor);
                        repairOutcome = currentAnchorIndex;
                        acceptedPathHash = newPathHash;
                        acceptedConfigHash = candidateHash;
                        repairSuccess = true;
                        break;
                    }
                }
            }

            if (!repairSuccess && !oldPath.empty()) {
                newPath = sipp.solve(grid, oldPath.front(), oldPath.back(),
                    result.reservations.safeIntervalTable, 0, AStarSippSolver::GoalOccupation::Permanent);
                if (structurallyValid(newPath, oldPath.front(), oldPath.back()) &&
                    solver_utils::canAcceptNewPath(
                        result.reservations.vertex_agents,
                        result.reservations.goal_reservations,
                        newPath, activeAgentId, 0, maxNewConflicts)) {
                    const Hash128 newPathHash = hashPath(newPath, instance.getNumCols());
                    const Hash128 candidateHash = configHash ^
                        agentContribution(activeIndex, pathHashes[activeIndex]) ^
                        agentContribution(activeIndex, newPathHash);
                    if (visitedConfigurations.contains(candidateHash)) {
                        ++sameConfigCounter;
                    } else {
                        acceptedPathHash = newPathHash;
                        acceptedConfigHash = candidateHash;
                        repairSuccess = true;
                        repairStartIndex = 0;
                        repairOutcome = AGENT_FULL_REPLAN;
                    }
                }
            }
            if (!repairSuccess) {
                // Restore before returning: result may be moved before local
                // destructors run when named return value optimization is absent.
                repair.rollback();
                result.agentAnchorsIndex[activeAgentId] = AGENT_REPAIR_FAILED;
                result.sameConfigCounter = sameConfigCounter;
                solver_utils::updateResultMetrics(result, agents, false, startedAt);
                return result;
            }

            visitedConfigurations.insert(acceptedConfigHash);
            repair.insertPath(newPath, activeAgentId);

            UpdateSolutionConflictsV2(
                newPath, result.reservations, result.remainingConflicts,
                static_cast<int>(activeIndex), repairStartIndex);
            // All allocating work finished. Publishing cannot leave paths and
            // reservations inconsistent if an earlier operation throws.
            static_assert(std::is_nothrow_move_assignable_v<decltype(newPath)>);
            result.paths[activeIndex] = std::move(newPath);
            repair.commit();
            result.agentAnchorsIndex[activeAgentId] = repairOutcome;
            pathHashes[activeIndex] = acceptedPathHash;
            configHash = acceptedConfigHash;
        }

        result.sameConfigCounter = sameConfigCounter;
        solver_utils::updateResultMetrics(result, agents, true, startedAt);
        return result;
    }

}
