#include "experiment_utils.hpp"

#include "mapf/utils.hpp"

#include <cctype>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <list>
#include <sstream>

#ifndef MAPF_REPOSITORY_ROOT
#define MAPF_REPOSITORY_ROOT "."
#endif

namespace mapf::experiments {

    namespace {
        std::string sanitizeFilenameComponent(const std::string& input) {
            std::string output;

            for (char character : input) {
                unsigned char unsignedCharacter = static_cast<unsigned char>(character);
                if (
                    std::isalnum(unsignedCharacter) ||
                    character == '.' ||
                    character == '_' ||
                    character == '-'
                ) {
                    output += character;
                } else {
                    output += '_';
                }
            }

            if (output.empty()) {
                return "unknown";
            }

            return output;
        }

        std::string currentBranchName() {
            std::ifstream headFile(repositoryRoot() / ".git" / "HEAD");
            if (!headFile.is_open()) {
                return "unknown";
            }

            std::string head;
            std::getline(headFile, head);

            constexpr const char* refPrefix = "ref: refs/heads/";
            std::string prefix(refPrefix);

            if (head.rfind(prefix, 0) == 0) {
                return sanitizeFilenameComponent(head.substr(prefix.size()));
            }

            return "detached";
        }

        std::string formatDouble(double value) {
            std::ostringstream output;
            output << std::fixed << std::setprecision(6) << value;
            return output.str();
        }
    }

    std::filesystem::path repositoryRoot() {
        return std::filesystem::path(MAPF_REPOSITORY_ROOT);
    }

    std::filesystem::path makeResultPath() {
        const auto now = std::chrono::system_clock::now();
        const auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(
            now.time_since_epoch()
        ).count();

        std::ostringstream filename;
        filename << currentBranchName() << "_" << timestamp << "_results.csv";

        return repositoryRoot() / "results" / filename.str();
    }

    double elapsedSeconds(std::chrono::steady_clock::time_point startedAt) {
        const auto finishedAt = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(finishedAt - startedAt).count();
    }

    bool writeExperimentResult(const Instance& instance, const Result& result, double experimentTimeSeconds) {
        std::list<std::string> headers = {
            "map",
            "instance_name",
            "num_agents",
            "success",
            "sumOfCosts",
            "makespan",
            "injustice",
            "durationSeconds",
            "time"
        };

        std::list<std::string> rowValues = {
            instance.getMapName(),
            instance.getInstanceName(),
            std::to_string(instance.getNumAgents()),
            result.success ? "true" : "false",
            std::to_string(result.sumOfCosts),
            std::to_string(result.makespan),
            formatDouble(result.injustice),
            formatDouble(result.durationSeconds),
            formatDouble(experimentTimeSeconds)
        };

        return writeResultsToCsvFile(makeResultPath().string(), headers, rowValues);
    }

}
