#include "experiment_utils.hpp"

#include "mapf/core/agent.hpp"
#include "mapf/core/instance.hpp"
#include "mapf/core/result.hpp"
#include "mapf/solvers/priority_planning_solver.hpp"

#include <chrono>
#include <list>
#include <vector>

int main() {
    const auto startedAt = std::chrono::steady_clock::now();

    std::vector<std::vector<int>> freeCells = {
        {1, 1, 1, 1, 1},
        {1, 0, 0, 0, 1},
        {1, 1, 1, 1, 1},
        {1, 0, 0, 0, 1},
        {1, 1, 1, 1, 1}
    };

    std::vector<mapf::Agent> agents = {
        {
            .id = 0,
            .currentPosition = {.x = 0, .y = 0},
            .startPosition = {.x = 0, .y = 0},
            .goalPosition = {.x = 4, .y = 4}
        },
        {
            .id = 1,
            .currentPosition = {.x = 4, .y = 0},
            .startPosition = {.x = 4, .y = 0},
            .goalPosition = {.x = 0, .y = 4}
        },
        {
            .id = 2,
            .currentPosition = {.x = 0, .y = 2},
            .startPosition = {.x = 0, .y = 2},
            .goalPosition = {.x = 4, .y = 2}
        }
    };

    mapf::Instance instance(&freeCells, 5, 5, agents);
    mapf::PriorityPlanningSolver solver(instance);
    mapf::Result result = solver.solve({0, 2, 1});

    const double experimentTimeSeconds = mapf::experiments::elapsedSeconds(startedAt);
    bool wroteResults = mapf::experiments::writeExperimentResult(instance, result, experimentTimeSeconds);

    return result.success && wroteResults ? 0 : 1;
}
