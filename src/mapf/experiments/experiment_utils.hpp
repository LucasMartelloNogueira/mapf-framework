#pragma once

#include "mapf/core/cell.hpp"
#include "mapf/core/instance.hpp"
#include "mapf/core/result.hpp"

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <list>
#include <string>
#include <vector>

namespace mapf::experiments {

    enum class ConflictType {
        Vertex,
        Edge
    };

    struct ConflictRecord {
        Cell* cell1;
        Cell* cell2;
        int timestep;
        ConflictType type;
        std::vector<int> agentIds;
    };

    struct ExperimentRunResult {
        Result metrics;
        std::vector<std::list<Cell*>> initialPaths;
        std::vector<std::list<Cell*>> solutionPaths;
        std::vector<ConflictRecord> remainingConflicts;
        int numAgents = 0;
        std::string solver;
        bool continueIfFailed = false;
        bool multithreading = false;
        std::size_t numThreads = 1;
        bool localRepair = false;
    };

    struct ExperimentOutputPaths {
        std::string prefix;
        std::filesystem::path directory;
        std::filesystem::path stats;
        std::filesystem::path solution;
        std::filesystem::path conflicts;
    };

    std::filesystem::path repositoryRoot();
    std::filesystem::path makeResultPath();
    ExperimentOutputPaths makeExperimentOutputPaths();
    double elapsedSeconds(std::chrono::steady_clock::time_point startedAt);

    std::vector<ConflictRecord> normalizeConflicts(
        const Instance& instance,
        const std::vector<std::list<Cell*>>& paths
    );

    bool writeExperimentArtifacts(
        const Instance& instance,
        const ExperimentRunResult& run,
        double experimentTimeSeconds
    );

    bool writeExperimentResult(
        const Instance& instance,
        const Result& result,
        double experimentTimeSeconds
    );

}
