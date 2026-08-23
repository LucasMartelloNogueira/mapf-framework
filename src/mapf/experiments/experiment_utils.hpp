#pragma once

#include "mapf/core/instance.hpp"
#include "mapf/core/result.hpp"

#include <chrono>
#include <filesystem>
#include <string>

namespace mapf::experiments {

    std::filesystem::path repositoryRoot();
    std::filesystem::path makeResultPath();
    double elapsedSeconds(std::chrono::steady_clock::time_point startedAt);
    bool writeExperimentResult(const Instance& instance, const Result& result, double experimentTimeSeconds);

}
