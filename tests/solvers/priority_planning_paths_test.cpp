#include "mapf/core/instance.hpp"
#include "mapf/solvers/priority_planning_solver.hpp"
#include "test_support.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <list>
#include <string>
#include <vector>

namespace {
    mapf::Agent agent(int id, int startX, int startY, int goalX, int goalY) {
        mapf::Position start {.x = startX, .y = startY};
        mapf::Position goal {.x = goalX, .y = goalY};
        return {
            .id = id,
            .currentPosition = start,
            .startPosition = start,
            .goalPosition = goal
        };
    }
}

int main() {
    // Scenario: two scenario rows use the same Moving AI bucket. Expected: unique experiment IDs coexist with the faithfully retained bucket values.
    {
        const std::filesystem::path directory =
            std::filesystem::temp_directory_path() /
            ("mapf_bucket_test_" + std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()
            ));
        std::filesystem::create_directories(directory);
        const std::filesystem::path mapPath = directory / "bucket-test.map";
        const std::filesystem::path scenarioPath = directory / "bucket-test.scen";

        {
            std::ofstream mapOutput(mapPath);
            mapOutput << "type octile\nheight 2\nwidth 3\nmap\n...\n...\n";
        }
        {
            std::ofstream scenarioOutput(scenarioPath);
            scenarioOutput
                << "version 1\n"
                << "7 bucket-test.map 3 2 0 0 2 0 2\n"
                << "7 bucket-test.map 3 2 0 1 2 1 2\n";
        }

        mapf::Instance instance(mapPath.string(), scenarioPath.string(), 2);
        requireTest(instance.getAgents()[0].id == 0, "First experiment ID is wrong.");
        requireTest(instance.getAgents()[1].id == 1, "Second experiment ID is wrong.");
        requireTest(instance.getAgents()[0].scenarioId == 7, "First bucket was lost.");
        requireTest(instance.getAgents()[1].scenarioId == 7, "Repeated bucket was lost.");

        std::filesystem::remove_all(directory);
    }

    // Scenario: two independent agents are solved in reverse priority order. Expected: exposed path vectors remain in instance order and manual buckets default to -1.
    {
        std::vector<std::vector<int>> freeCells(2, std::vector<int>(3, 1));
        std::vector<mapf::Agent> agents {
            agent(10, 0, 0, 2, 0),
            agent(20, 2, 1, 0, 1)
        };
        mapf::Instance instance(&freeCells, 2, 3, agents);
        mapf::PriorityPlanningSolver solver(instance);
        mapf::Result result = solver.solve({20, 10});

        requireTest(result.success, "Reverse priority order failed.");
        requireTest(solver.getInitialPaths().size() == 2, "Initial paths are not aligned.");
        requireTest(solver.getPaths().size() == 2, "Solution paths are not aligned.");
        requireTest(
            solver.getPaths()[0].front()->position.x == agents[0].startPosition.x &&
                solver.getPaths()[0].front()->position.y == agents[0].startPosition.y,
            "Agent 10 path was reordered."
        );
        requireTest(
            solver.getPaths()[1].front()->position.x == agents[1].startPosition.x &&
                solver.getPaths()[1].front()->position.y == agents[1].startPosition.y,
            "Agent 20 path was reordered."
        );
        requireTest(instance.getAgents()[0].scenarioId == -1, "Manual bucket default is wrong.");

        mapf::Result invalid = solver.solve({10, 10});
        requireTest(!invalid.success, "Duplicate priority IDs were accepted.");
        requireTest(solver.getPaths()[0].empty(), "Repeated solve retained an old path.");
        requireTest(solver.getInitialPaths()[0].empty(), "Repeated solve retained an old initial path.");
    }

    // Scenario: the second priority item shares the first agent's permanent goal. Expected: the first committed path and partial metrics survive the failure.
    {
        std::vector<std::vector<int>> freeCells(2, std::vector<int>(3, 1));
        std::vector<mapf::Agent> agents {
            agent(10, 0, 0, 2, 0),
            agent(20, 0, 1, 2, 0)
        };
        mapf::Instance instance(&freeCells, 2, 3, agents);
        mapf::PriorityPlanningSolver solver(instance);
        mapf::Result result = solver.solve({10, 20});

        requireTest(!result.success, "Duplicate permanent goals unexpectedly succeeded.");
        requireTest(!solver.getPaths()[0].empty(), "The first committed path was lost.");
        requireTest(solver.getPaths()[1].empty(), "The failed second path was committed.");
        requireTest(result.sumOfCosts == 2, "Partial sum of costs is wrong.");
        requireTest(result.makespan == 2, "Partial makespan is wrong.");
    }

    return 0;
}
