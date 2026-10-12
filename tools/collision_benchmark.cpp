// Self-contained differential oracle and benchmark; not linked into libmapf.
#include "mapf/core/grid.hpp"
#include "mapf/utils.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

namespace {
    struct OracleKey {
        int time;
        int kind;
        int x1;
        int y1;
        int x2;
        int y2;
        bool operator==(const OracleKey& other) const = default;
        bool operator<(const OracleKey& other) const {
            return std::tie(time, kind, x1, y1, x2, y2) <
                std::tie(other.time, other.kind, other.x1, other.y1, other.x2, other.y2);
        }
    };

    struct OracleEvent {
        std::set<int> participants;
        std::set<int> forward;
        std::set<int> reverse;
        bool operator==(const OracleEvent& other) const = default;
    };

    struct OracleSnapshot {
        std::map<OracleKey, OracleEvent> events;
        std::vector<std::vector<OracleKey>> byAgent;
        bool operator==(const OracleSnapshot& other) const = default;
    };

    struct Measurements {
        double median;
        double minimum;
        double maximum;
    };

    OracleKey keyFor(mapf::Cell* first, mapf::Cell* second, int time, int kind) {
        // Independent ordering, without production makeEdgeTime/comparator.
        if (std::tie(second->position.x, second->position.y) <
            std::tie(first->position.x, first->position.y)) {
            std::swap(first, second);
        }
        return {time, kind, first->position.x, first->position.y, second->position.x, second->position.y};
    }

    OracleSnapshot pairwiseOracle(const std::vector<std::list<mapf::Cell*>>& paths) {
        OracleSnapshot result;
        result.byAgent.resize(paths.size());
        std::vector<std::vector<mapf::Cell*>> indexed;
        std::size_t horizon = 0;
        for (const std::list<mapf::Cell*>& path : paths) {
            indexed.emplace_back(path.begin(), path.end());
            horizon = std::max(horizon, path.size());
        }
        for (std::size_t time = 0; time < horizon; ++time) {
            for (std::size_t i = 0; i < indexed.size(); ++i) {
                if (indexed[i].empty()) {
                    continue;
                }
                mapf::Cell* first = indexed[i][std::min(time, indexed[i].size() - 1)];
                for (std::size_t j = i + 1; j < indexed.size(); ++j) {
                    if (indexed[j].empty()) {
                        continue;
                    }
                    mapf::Cell* second = indexed[j][std::min(time, indexed[j].size() - 1)];
                    if (first == second) {
                        OracleEvent& event = result.events[keyFor(first, first, static_cast<int>(time), 0)];
                        event.participants.insert(static_cast<int>(i));
                        event.participants.insert(static_cast<int>(j));
                    }
                    if (time == 0) {
                        continue;
                    }
                    mapf::Cell* previousFirst = indexed[i][std::min(time - 1, indexed[i].size() - 1)];
                    mapf::Cell* previousSecond = indexed[j][std::min(time - 1, indexed[j].size() - 1)];
                    if (previousFirst != first && previousSecond != second &&
                        previousFirst == second && previousSecond == first) {
                        const OracleKey key = keyFor(previousFirst, first, static_cast<int>(time), 1);
                        OracleEvent& event = result.events[key];
                        event.participants.insert(static_cast<int>(i));
                        event.participants.insert(static_cast<int>(j));
                        if (previousFirst->position.x == key.x1 && previousFirst->position.y == key.y1) {
                            event.forward.insert(static_cast<int>(i));
                            event.reverse.insert(static_cast<int>(j));
                        } else {
                            event.reverse.insert(static_cast<int>(i));
                            event.forward.insert(static_cast<int>(j));
                        }
                    }
                }
            }
        }
        for (const std::pair<const OracleKey, OracleEvent>& entry : result.events) {
            for (int owner : entry.second.participants) {
                if (static_cast<std::size_t>(entry.first.time) < indexed[static_cast<std::size_t>(owner)].size()) {
                    result.byAgent[static_cast<std::size_t>(owner)].push_back(entry.first);
                }
            }
        }
        return result;
    }

    OracleSnapshot canonicalSnapshot(const mapf::SolutionConflicts& snapshot) {
        OracleSnapshot result;
        result.byAgent.resize(snapshot.byAgent.size());
        for (const std::pair<const mapf::CellTime, mapf::VertexEvent>& entry : snapshot.vertexEvents) {
            result.events[keyFor(entry.first.cell, entry.first.cell, entry.first.time, 0)].participants.insert(
                entry.second.participants.begin(), entry.second.participants.end());
        }
        for (const std::pair<const mapf::EdgeTime, mapf::EdgeEvent>& entry : snapshot.edgeEvents) {
            OracleEvent& event = result.events[keyFor(entry.first.first, entry.first.second, entry.first.time, 1)];
            event.forward.insert(entry.second.forward.begin(), entry.second.forward.end());
            event.reverse.insert(entry.second.reverse.begin(), entry.second.reverse.end());
            event.participants.insert(event.forward.begin(), event.forward.end());
            event.participants.insert(event.reverse.begin(), event.reverse.end());
        }
        for (std::size_t i = 0; i < snapshot.byAgent.size(); ++i) {
            for (const std::variant<mapf::CellConflict, mapf::EdgeConflict>& record : snapshot.byAgent[i]) {
                const mapf::CellConflict* vertex = std::get_if<mapf::CellConflict>(&record);
                if (vertex != nullptr) {
                    result.byAgent[i].push_back(keyFor(vertex->cell, vertex->cell, vertex->time, 0));
                } else {
                    const mapf::EdgeConflict& edge = std::get<mapf::EdgeConflict>(record);
                    result.byAgent[i].push_back(keyFor(edge.cell_1, edge.cell_2, edge.time, 1));
                }
            }
        }
        return result;
    }

    void compare(const std::vector<std::list<mapf::Cell*>>& paths, const std::string& name) {
        if (canonicalSnapshot(getCollision(paths)) != pairwiseOracle(paths)) {
            throw std::runtime_error("Oracle mismatch: " + name);
        }
    }

    void walkTo(mapf::Grid& grid, std::list<mapf::Cell*>& path, int x, int y) {
        int currentX = path.back()->position.x;
        int currentY = path.back()->position.y;
        while (currentX != x) {
            currentX += currentX < x ? 1 : -1;
            path.push_back(grid.getCellPtr(currentX, currentY));
        }
        while (currentY != y) {
            currentY += currentY < y ? 1 : -1;
            path.push_back(grid.getCellPtr(currentX, currentY));
        }
    }

    std::vector<std::list<mapf::Cell*>> randomWalks(mapf::Grid& grid, std::uint32_t seed, int count) {
        std::mt19937 random(seed);
        std::set<mapf::Cell*> starts;
        std::set<mapf::Cell*> goals;
        std::vector<std::list<mapf::Cell*>> paths;
        for (int i = 0; i < count; ++i) {
            int x = 0;
            int y = 0;
            do {
                x = static_cast<int>(random() % 16);
                y = static_cast<int>(random() % 16);
            } while (!starts.insert(grid.getCellPtr(x, y)).second);
            std::list<mapf::Cell*> path{grid.getCellPtr(x, y)};
            const int length = static_cast<int>(random() % 100) + 1;
            for (int step = 0; step < length || goals.contains(path.back()); ++step) {
                const int direction = static_cast<int>(random() % 5);
                if (direction == 0 && x > 0) { --x; }
                if (direction == 1 && x < 15) { ++x; }
                if (direction == 2 && y > 0) { --y; }
                if (direction == 3 && y < 15) { ++y; }
                path.push_back(grid.getCellPtr(x, y));
            }
            goals.insert(path.back());
            paths.push_back(std::move(path));
            if (i % 7 == 0) {
                paths.emplace_back();
            }
        }
        return paths;
    }

    std::vector<std::list<mapf::Cell*>> workload(mapf::Grid& grid, const std::string& name, std::uint32_t seed) {
        if (name == "random") {
            return randomWalks(grid, seed, 192);
        }
        std::vector<std::list<mapf::Cell*>> paths;
        const int count = name == "lanes_large" ? 1024 : 256;
        for (int i = 0; i < count; ++i) {
            std::list<mapf::Cell*> path;
            if (name == "lanes" || name == "lanes_large") {
                path.push_back(grid.getCellPtr(0, i));
                walkTo(grid, path, 127, i);
            } else if (name == "dense") {
                const int x = 24 + i % 16;
                const int y = 24 + i / 16;
                path.push_back(grid.getCellPtr(x, y));
                const int distance = 128 - x - y;
                path.insert(path.end(), static_cast<std::size_t>(128 - distance), path.front());
                walkTo(grid, path, 64, 64);
                walkTo(grid, path, x, y);
            } else if (name == "parked") {
                if (i < 200) {
                    path.push_back(grid.getCellPtr(i, 1));
                } else {
                    path.push_back(grid.getCellPtr(0, i));
                    walkTo(grid, path, 0, 1);
                    walkTo(grid, path, 220, 1);
                    walkTo(grid, path, 220, i);
                }
            } else {
                throw std::invalid_argument("Unknown workload: " + name);
            }
            paths.push_back(std::move(path));
        }
        return paths;
    }

    std::size_t consume(const mapf::SolutionConflicts& snapshot) {
        std::size_t count = snapshot.vertexEvents.size() + snapshot.edgeEvents.size();
        for (const std::vector<std::variant<mapf::CellConflict, mapf::EdgeConflict>>& records : snapshot.byAgent) {
            count += records.size();
        }
        return count;
    }

    std::size_t consume(const OracleSnapshot& snapshot) {
        std::size_t count = snapshot.events.size();
        for (const std::vector<OracleKey>& records : snapshot.byAgent) {
            count += records.size();
        }
        return count;
    }

    template<class Detector>
    Measurements measure(Detector detector, int repetitions, std::size_t& checksum) {
        checksum += consume(detector()); // warmup
        std::vector<double> samples;
        for (int iteration = 0; iteration < repetitions; ++iteration) {
            const std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
            checksum += consume(detector());
            samples.push_back(std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started).count());
        }
        std::sort(samples.begin(), samples.end());
        return {samples[samples.size() / 2], samples.front(), samples.back()};
    }

    void verifyExamples(mapf::Grid& grid) {
        mapf::Cell* left = grid.getCellPtr(0, 1);
        mapf::Cell* center = grid.getCellPtr(1, 1);
        mapf::Cell* right = grid.getCellPtr(2, 1);
        mapf::Cell* up = grid.getCellPtr(1, 0);
        mapf::Cell* down = grid.getCellPtr(1, 2);
        compare({}, "empty");
        compare({{left, left, center, left}}, "self revisit and wait");
        compare({{center}, {}, {left, left, center, right}, {up, up, center, down}}, "two visitors parked");
        compare({{left, left, center, right}, {}, {up, up, center, down}, {center}}, "parked last");
        compare({{left, center}, {up, center, down}}, "arrival equality");
        compare({{left, center, right}, {up, center, down}, {right, center, left}, {down, center, up}}, "four occupants");
        compare({{left, center, right}, {center, left, grid.getCellPtr(0, 0), up}}, "swap");
        compare({{left, center}, {center, right}}, "following");
        mapf::Cell* corner = grid.getCellPtr(0, 0);
        compare({{left, center, up}, {center, up, corner}, {up, corner, left}, {corner, left, center}}, "cycle");
        // Three forward versus one reverse: distinct full endpoints, common
        // interior transitions; removing the reverse leaves only vertex events.
        const std::vector<std::list<mapf::Cell*>> mixed {
            {grid.getCellPtr(0, 0), left, center, right},
            {grid.getCellPtr(0, 2), left, center, up},
            {left, left, center, down},
            {right, center, left}
        };
        compare(mixed, "three versus one directions");
        std::vector<std::list<mapf::Cell*>> removed = mixed;
        removed.back().clear();
        compare(removed, "reverse removed");
        const std::vector<std::vector<std::list<mapf::Cell*>>> invalid {
            {{left, center}, {left, right}},
            {{left, center}, {right, center}},
            {{left, nullptr, center}}
        };
        for (const std::vector<std::list<mapf::Cell*>>& paths : invalid) {
            bool rejected = false;
            try { getCollision(paths); } catch (const std::invalid_argument&) { rejected = true; }
            if (!rejected || validateSolution(paths)) {
                throw std::runtime_error("Invalid detector input accepted.");
            }
        }
    }
}

int main(int argc, char* argv[]) {
    try {
        const std::string name = argc > 1 ? argv[1] : "all";
        const int repetitions = argc > 2 ? std::stoi(argv[2]) : 7;
        const std::uint32_t seed = 73421;
        if (repetitions < 1) {
            throw std::invalid_argument("Repetitions must be positive.");
        }
        mapf::Grid grid(1024, 256);
        verifyExamples(grid);
        for (std::uint32_t offset = 0; offset < 100; ++offset) {
            compare(randomWalks(grid, seed + offset, 24), "seed=" + std::to_string(seed + offset));
        }
        std::cout << "compiler=" << __VERSION__ << " build=" << MAPF_BENCHMARK_BUILD
            << " flags=" << MAPF_BENCHMARK_FLAGS << " seed=" << seed << " differential_random_cases=100\n";
        std::ifstream cpu("/proc/cpuinfo");
        std::string line;
        while (std::getline(cpu, line)) {
            if (line.rfind("model name", 0) == 0) { std::cout << line << '\n'; break; }
        }
        std::size_t checksum = 0;
        const std::vector<std::string> names = name == "all"
            ? std::vector<std::string>{"lanes", "lanes_large", "dense", "parked", "random"}
            : std::vector<std::string>{name};
        for (const std::string& selected : names) {
            const std::vector<std::list<mapf::Cell*>> paths = workload(grid, selected, seed);
            compare(paths, selected);
            const mapf::SolutionConflicts snapshot = getCollision(paths);
            std::size_t cells = 0;
            std::size_t horizon = 0;
            for (const std::list<mapf::Cell*>& path : paths) {
                cells += path.size();
                horizon = std::max(horizon, path.size());
            }
            const Measurements indexed = measure([&paths]() { return getCollision(paths); }, repetitions, checksum);
            const Measurements pairwise = measure([&paths]() { return pairwiseOracle(paths); }, repetitions, checksum);
            std::cout << selected << " A=" << paths.size() << " S=" << cells << " H=" << horizon - 1
                << " vertex=" << snapshot.vertexEvents.size() << " edge=" << snapshot.edgeEvents.size()
                << " records=" << consume(snapshot) - snapshot.vertexEvents.size() - snapshot.edgeEvents.size()
                << " indexed_ms=" << indexed.median << " [" << indexed.minimum << ',' << indexed.maximum << ']'
                << " oracle_ms=" << pairwise.median << " [" << pairwise.minimum << ',' << pairwise.maximum << ']'
                << " speedup=" << pairwise.median / indexed.median << '\n';
        }
        std::ifstream memory("/proc/self/status");
        while (std::getline(memory, line)) {
            if (line.rfind("VmHWM:", 0) == 0) { std::cout << "process_peak_" << line << '\n'; }
        }
        std::cout << "checksum=" << checksum << '\n';
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
