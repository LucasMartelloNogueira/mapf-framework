#include "experiment_utils.hpp"

#include "mapf/core/instance.hpp"
#include "mapf/core/result.hpp"
#include "mapf/solvers/priority_planning_solver.hpp"

#include <chrono>
#include <filesystem>

int main() {
    const auto startedAt = std::chrono::steady_clock::now();

    const std::filesystem::path root = mapf::experiments::repositoryRoot();
    const std::filesystem::path mapPath = root / "benchmarks" / "maps" / "empty-8-8.map";
    const std::filesystem::path scenarioPath = root / "benchmarks" / "scenarios" / "empty-8-8" / "random" / "empty-8-8-random-1.scen";

    mapf::Instance instance(mapPath.string(), scenarioPath.string(), 3);
    mapf::PriorityPlanningSolver solver(instance);
    mapf::Result result = solver.solve({0, 1, 2});

    const double experimentTimeSeconds = mapf::experiments::elapsedSeconds(startedAt);
    bool wroteResults = mapf::experiments::writeExperimentResult(instance, result, experimentTimeSeconds);

    return result.success && wroteResults ? 0 : 1;
}
