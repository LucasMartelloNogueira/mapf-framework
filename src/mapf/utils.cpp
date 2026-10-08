#include "mapf/utils.hpp"
#include "mapf/solvers/local_path_repair_solver.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <string>
#include <vector>

namespace {
    struct GoalOccupancy {
        int i;
        int arrivalTime;
    };

    std::string escapeCsvField(const std::string& field) {
        bool mustQuote = field.find_first_of(",\"\r\n") != std::string::npos;
        if (!mustQuote) {
            return field;
        }

        std::string escaped = "\"";

        for (char character : field) {
            if (character == '"') {
                escaped += "\"\"";
            } else {
                escaped += character;
            }
        }

        escaped += "\"";
        return escaped;
    }

    void writeCsvRow(std::ofstream& output, const CsvRow& fields) {
        bool first = true;

        for (const std::string& field : fields) {
            if (!first) {
                output << ",";
            }

            output << escapeCsvField(field);
            first = false;
        }

        output << "\n";
    }
}

void printPath(const std::list<mapf::Cell*>& path) {
    for (mapf::Cell* cell : path) {
        mapf::printCell(cell);
    }
}

bool validateSolution(const std::vector<std::list<mapf::Cell*>>& paths) {
    try {
        return getCollision(paths).empty();
    } catch (const std::invalid_argument&) {
        return false;
    }
}

mapf::SolutionConflicts getCollision(const std::vector<std::list<mapf::Cell*>>& paths) {
    if (paths.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument("Path indexes must fit in int.");
    }

    mapf::SolutionConflicts conflicts;
    conflicts.byAgent.resize(paths.size());
    std::unordered_map<mapf::Cell*, int> startOwners;
    std::unordered_map<mapf::Cell*, GoalOccupancy> goalVertexLookup;
    std::vector<int> arrivalTimes(paths.size(), -1);
    startOwners.reserve(paths.size());
    goalVertexLookup.reserve(paths.size());
    std::size_t explicitCells = 0;
    std::size_t moves = 0;
    int i = 0;

    // Validate every pointer before hashing coordinates, and register all goals
    // before reading occupancy so parked owners are independent of path order.
    for (const std::list<mapf::Cell*>& path : paths) {
        if (!path.empty()) {
            if (path.size() - 1 > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
                path.size() > std::numeric_limits<std::size_t>::max() - explicitCells) {
                throw std::invalid_argument("Path time or total size is unrepresentable.");
            }
            explicitCells += path.size();
            mapf::Cell* previous = nullptr;
            for (mapf::Cell* cell : path) {
                if (cell == nullptr) {
                    throw std::invalid_argument("Paths must not contain null cells.");
                }
                if (previous != nullptr && previous != cell) {
                    ++moves; // At most explicitCells, whose sum was checked above.
                }
                previous = cell;
            }
            const int arrival = static_cast<int>(path.size() - 1);
            if (!startOwners.emplace(path.front(), i).second ||
                !goalVertexLookup.emplace(path.back(), GoalOccupancy{i, arrival}).second) {
                throw std::invalid_argument("Path starts and path goals must each be unique.");
            }
            arrivalTimes[static_cast<std::size_t>(i)] = arrival;
        }
        ++i;
    }

    std::unordered_map<mapf::CellTime, mapf::VertexEvent, mapf::CellTimeHash> vertexLookup;
    std::unordered_map<mapf::EdgeTime, mapf::EdgeEvent, mapf::EdgeTimeHash> edgeLookup;
    vertexLookup.reserve(explicitCells);
    edgeLookup.reserve(moves);
    i = 0;
    for (const std::list<mapf::Cell*>& path : paths) {
        std::size_t position = 0;
        mapf::Cell* previousCell = nullptr;
        for (mapf::Cell* cell : path) {
            const int time = static_cast<int>(position);
            std::unordered_set<int>& occupants = vertexLookup[{cell, time}].participants;
            occupants.insert(i);
            
            const std::unordered_map<mapf::Cell*, GoalOccupancy>::const_iterator goal =
                goalVertexLookup.find(cell);

            if (goal != goalVertexLookup.cend() && goal->second.i != i &&
                goal->second.arrivalTime < time) {
                occupants.insert(goal->second.i);
            }
            if (previousCell != nullptr && previousCell != cell) {
                const mapf::EdgeTime key = mapf::makeEdgeTime(previousCell, cell, time);
                mapf::EdgeEvent& event = edgeLookup[key];
                if (previousCell == key.first) {
                    event.forward.insert(i);
                } else {
                    event.reverse.insert(i);
                }
            }
            previousCell = cell;
            ++position;
        }
        ++i;
    }

    for (std::pair<const mapf::CellTime, mapf::VertexEvent>& entry : vertexLookup) {
        if (entry.second.participants.size() < 2) {
            continue;
        }
        for (int owner : entry.second.participants) {
            if (entry.first.time <= arrivalTimes[static_cast<std::size_t>(owner)]) {
                conflicts.byAgent[static_cast<std::size_t>(owner)].push_back(
                    mapf::CellConflict{entry.first.cell, entry.first.time});
            }
        }
        conflicts.vertexEvents.emplace(entry.first, std::move(entry.second));
    }
    for (std::pair<const mapf::EdgeTime, mapf::EdgeEvent>& entry : edgeLookup) {
        if (!entry.second.active()) {
            continue;
        }
        const mapf::EdgeConflict record{entry.first.first, entry.first.second, entry.first.time};
        for (int owner : entry.second.forward) {
            conflicts.byAgent[static_cast<std::size_t>(owner)].push_back(record);
        }
        for (int owner : entry.second.reverse) {
            conflicts.byAgent[static_cast<std::size_t>(owner)].push_back(record);
        }
        conflicts.edgeEvents.emplace(entry.first, std::move(entry.second));
    }
    for (std::vector<std::variant<mapf::CellConflict, mapf::EdgeConflict>>& records : conflicts.byAgent) {
        std::sort(records.begin(), records.end(), mapf::conflictRecordComesBefore);
    }
    return conflicts;
}

mapf::SolutionConflicts updateSolutionConflicts(
    std::list<mapf::Cell*> oldPath,
    std::list<mapf::Cell*> newPath,
    int index,
    std::variant<mapf::CellConflict, mapf::EdgeConflict> conflict,
    mapf::SolutionConflicts conflicts
) {
    if (index < 0 || static_cast<std::size_t>(index) >= conflicts.byAgent.size()) {
        throw std::invalid_argument("The agent index is outside the conflict records.");
    }
    if (oldPath.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
        newPath.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument("Path time is unrepresentable.");
    }

    auto& agentConflicts = conflicts.byAgent[index];
    const auto matchesConflict = [&conflict](const auto& record) {
        if (record.index() != conflict.index()) {
            return false;
        }
        if (const auto* vertex = std::get_if<mapf::CellConflict>(&record)) {
            const auto& target = std::get<mapf::CellConflict>(conflict);
            return vertex->cell == target.cell && vertex->time == target.time;
        }
        const auto& edge = std::get<mapf::EdgeConflict>(record);
        const auto& target = std::get<mapf::EdgeConflict>(conflict);
        return edge.cell_1 == target.cell_1 && edge.cell_2 == target.cell_2 &&
            edge.time == target.time;
    };
    if (std::count_if(agentConflicts.begin(), agentConflicts.end(), matchesConflict) != 1) {
        throw std::invalid_argument("The conflict must occur exactly once for the agent.");
    }

    int t = 0;
    mapf::Cell* previous = nullptr;
    for (mapf::Cell* cell : oldPath) {
        if (cell == nullptr) {
            throw std::invalid_argument("Paths must not contain null cells.");
        }
        const mapf::CellTime cellTime {cell, t};
        const auto vertex = conflicts.vertexEvents.find(cellTime);
        if (vertex != conflicts.vertexEvents.end()) {
            vertex->second.participants.erase(index);
        }
        if (t > 0 && previous != cell) {
            const mapf::EdgeTime edgeTime = mapf::makeEdgeTime(previous, cell, t);
            const auto edge = conflicts.edgeEvents.find(edgeTime);
            if (edge != conflicts.edgeEvents.end()) {
                if (previous == edgeTime.first) {
                    edge->second.forward.erase(index);
                } else {
                    edge->second.reverse.erase(index);
                }
            }
        }
        ++t;
        previous = cell;
    }

    t = 0;
    previous = nullptr;
    for (mapf::Cell* cell : newPath) {
        if (cell == nullptr) {
            throw std::invalid_argument("Paths must not contain null cells.");
        }
        const mapf::CellTime cellTime {cell, t};
        conflicts.vertexEvents[cellTime].participants.insert(index);
        if (t > 0 && previous != cell) {
            const mapf::EdgeTime edgeTime = mapf::makeEdgeTime(previous, cell, t);
            mapf::EdgeEvent& edge = conflicts.edgeEvents[edgeTime];
            if (previous == edgeTime.first) {
                edge.forward.insert(index);
            } else {
                edge.reverse.insert(index);
            }
        }
        ++t;
        previous = cell;
    }

    int i = 0;
    std::vector<std::variant<mapf::CellConflict, mapf::EdgeConflict>> remaining(agentConflicts.size() - 1);
    for (const auto& record : agentConflicts) {
        if (!matchesConflict(record)) {
            remaining[i] = record;
            ++i;
        } else if (const auto* vertex = std::get_if<mapf::CellConflict>(&conflict)) {
            const mapf::CellTime cellTime {vertex->cell, vertex->time};
            const auto event = conflicts.vertexEvents.find(cellTime);
            if (event != conflicts.vertexEvents.end() && event->second.participants.size() == 1) {
                const int remainingAgent = *event->second.participants.begin();
                if (remainingAgent != index) {
                    std::erase_if(conflicts.byAgent[remainingAgent], matchesConflict);
                }
            }
        } else {
            const auto& edge = std::get<mapf::EdgeConflict>(conflict);
            const mapf::EdgeTime edgeTime = mapf::makeEdgeTime(edge.cell_1, edge.cell_2, edge.time);
            const auto event = conflicts.edgeEvents.find(edgeTime);
            if (event != conflicts.edgeEvents.end() && !event->second.active()) {
                const auto& remainingAgents = event->second.forward.empty()
                    ? event->second.reverse : event->second.forward;
                for (int remainingAgent : remainingAgents) {
                    if (remainingAgent != index) {
                        std::erase_if(conflicts.byAgent[remainingAgent], matchesConflict);
                    }
                }
            }
        }
    }
    agentConflicts = std::move(remaining);
    return conflicts;
}

mapf::SolutionConflicts UpdateSolutionConflictsV2(
    const std::list<mapf::Cell*>& newPath,
    mapf::PathReservationState state,
    mapf::SolutionConflicts conflicts,
    int agentId,
    int pathIndex
) {
    if (agentId < 0 || static_cast<std::size_t>(agentId) >= conflicts.byAgent.size()) {
        throw std::invalid_argument("The agent ID must be a valid byAgent index.");
    }
    if (pathIndex < 0 || static_cast<std::size_t>(pathIndex) > newPath.size()) {
        throw std::invalid_argument("The path index is outside the path.");
    }
    if (newPath.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument("Path time is unrepresentable.");
    }
    const std::vector<mapf::Cell*> path(newPath.begin(), newPath.end());
    for (mapf::Cell* cell : path) {
        if (cell == nullptr) {
            throw std::invalid_argument("Paths must not contain null cells.");
        }
    }
    if (static_cast<std::size_t>(pathIndex) == path.size()) {
        return conflicts;
    }

    const auto explicitOccupantsAt = [&state](mapf::Cell* cell, int time) -> const std::unordered_set<int>* {
        const auto vertex = state.vertex_agents.find(cell);
        if (vertex == state.vertex_agents.end()) {
            return nullptr;
        }
        const auto occupants = vertex->second.find(time);
        return occupants == vertex->second.end() ? nullptr : &occupants->second;
    };
    const auto sameConflict = [](const auto& first, const auto& second) {
        return !mapf::conflictRecordComesBefore(first, second) &&
            !mapf::conflictRecordComesBefore(second, first);
    };
    const auto setAgentConflict = [&](int participant, const auto& record, bool present) {
        if (participant < 0 || static_cast<std::size_t>(participant) >= conflicts.byAgent.size()) {
            throw std::invalid_argument("Reservation agent IDs must be valid byAgent indexes.");
        }
        auto& records = conflicts.byAgent[participant];
        if (!present) {
            std::erase_if(records, [&](const auto& existing) { return sameConflict(existing, record); });
            return;
        }
        const auto position = std::lower_bound(records.begin(), records.end(), record, mapf::conflictRecordComesBefore);
        if (position == records.end() || !sameConflict(*position, record)) {
            records.insert(position, record);
        }
    };

    std::unordered_set<mapf::CellTime, mapf::CellTimeHash> affectedVertices;
    std::unordered_set<mapf::EdgeTime, mapf::EdgeTimeHash> affectedEdges;
    // Old records can refer to cells and edges no longer visited by the new path.
    for (const auto& record : conflicts.byAgent[agentId]) {
        if (const auto* vertex = std::get_if<mapf::CellConflict>(&record)) {
            if (vertex->time >= pathIndex) {
                affectedVertices.insert({vertex->cell, vertex->time});
            }
        } else {
            const auto& edge = std::get<mapf::EdgeConflict>(record);
            if (edge.time > pathIndex) {
                affectedEdges.insert(mapf::makeEdgeTime(edge.cell_1, edge.cell_2, edge.time));
            }
        }
    }
    // Parked owners participate in global events, but have no byAgent record
    // after their explicit path ends. Their previous events still need cleanup.
    for (const auto& [key, event] : conflicts.vertexEvents) {
        if (key.time >= pathIndex && event.participants.contains(agentId)) {
            affectedVertices.insert(key);
        }
    }
    for (std::size_t i = static_cast<std::size_t>(pathIndex); i < path.size(); ++i) {
        affectedVertices.insert({path[i], static_cast<int>(i)});
        // The prefix through pathIndex is unchanged; include the first modified movement.
        if (i > static_cast<std::size_t>(pathIndex) && path[i - 1] != path[i]) {
            affectedEdges.insert(mapf::makeEdgeTime(path[i - 1], path[i], static_cast<int>(i)));
        }
    }
    // Changing this agent's final arrival can affect other paths even beyond
    // its own explicit end. Scan the finite visits to its destination.
    const auto goalVisits = state.vertex_agents.find(path.back());
    if (goalVisits != state.vertex_agents.end()) {
        for (const auto& [time, participants] : goalVisits->second) {
            if (time >= pathIndex) {
                affectedVertices.insert({path.back(), time});
            }
        }
    }

    for (const auto& key : affectedVertices) {
        const mapf::CellConflict record{key.cell, key.time};
        const auto* explicitParticipants = explicitOccupantsAt(key.cell, key.time);
        std::unordered_set<int> participants = explicitParticipants == nullptr
            ? std::unordered_set<int>{} : *explicitParticipants;
        const auto goal = state.goal_reservations.find(key.cell);
        if (goal != state.goal_reservations.end() && key.time >= goal->second.arrivalTime) {
            participants.insert(goal->second.agentId);
        }

        const bool active = participants.size() > 1;
        std::unordered_set<int> affectedParticipants = participants;
        affectedParticipants.insert(agentId);
        const auto oldEvent = conflicts.vertexEvents.find(key);
        if (oldEvent != conflicts.vertexEvents.end()) {
            affectedParticipants.insert(oldEvent->second.participants.begin(), oldEvent->second.participants.end());
        }
        for (int participant : affectedParticipants) {
            // Virtual occupancy belongs to the global event only. The moving
            // participant owns the repair record after the parked path ends.
            setAgentConflict(participant, record, active && explicitParticipants != nullptr &&
                explicitParticipants->contains(participant));
        }
        if (active) {
            conflicts.vertexEvents.insert_or_assign(key, mapf::VertexEvent{std::move(participants)});
        } else {
            conflicts.vertexEvents.erase(key);
        }
    }

    for (const auto& key : affectedEdges) {
        const mapf::EdgeConflict record{key.first, key.second, key.time};
        const auto* firstBefore = explicitOccupantsAt(key.first, key.time - 1);
        const auto* secondNow = explicitOccupantsAt(key.second, key.time);
        const auto* secondBefore = explicitOccupantsAt(key.second, key.time - 1);
        const auto* firstNow = explicitOccupantsAt(key.first, key.time);
        mapf::EdgeEvent event;
        if (firstBefore != nullptr && secondNow != nullptr) {
            for (int participant : *firstBefore) {
                if (secondNow->contains(participant)) {
                    event.forward.insert(participant);
                }
            }
        }
        if (secondBefore != nullptr && firstNow != nullptr) {
            for (int participant : *secondBefore) {
                if (firstNow->contains(participant)) {
                    event.reverse.insert(participant);
                }
            }
        }
        std::unordered_set<int> affectedParticipants = event.forward;
        affectedParticipants.insert(event.reverse.begin(), event.reverse.end());
        affectedParticipants.insert(agentId);
        const auto oldEvent = conflicts.edgeEvents.find(key);
        if (oldEvent != conflicts.edgeEvents.end()) {
            affectedParticipants.insert(oldEvent->second.forward.begin(), oldEvent->second.forward.end());
            affectedParticipants.insert(oldEvent->second.reverse.begin(), oldEvent->second.reverse.end());
        }
        const bool active = event.active();
        for (int participant : affectedParticipants) {
            setAgentConflict(participant, record, active &&
                (event.forward.contains(participant) || event.reverse.contains(participant)));
        }
        if (active) {
            conflicts.edgeEvents.insert_or_assign(key, std::move(event));
        } else {
            conflicts.edgeEvents.erase(key);
        }
    }
    return conflicts;
}


bool writeRowsToCsvFile(
    const std::filesystem::path& outputPath,
    const CsvRow& headers,
    const std::vector<CsvRow>& rows
) {
    if (outputPath.extension() != ".csv") {
        return false;
    }

    for (const CsvRow& row : rows) {
        if (headers.size() != row.size()) {
            return false;
        }
    }

    std::filesystem::path parentPath = outputPath.parent_path();
    if (!parentPath.empty()) {
        std::error_code errorCode;
        std::filesystem::create_directories(parentPath, errorCode);
        if (errorCode) {
            return false;
        }
    }

    std::ofstream output(outputPath, std::ios::trunc);
    if (!output.is_open()) {
        return false;
    }

    writeCsvRow(output, headers);
    for (const CsvRow& row : rows) {
        writeCsvRow(output, row);
    }

    return output.good();
}

bool writeResultsToCsvFile(
    const std::string& outputFilename,
    const std::list<std::string>& headers,
    const std::list<std::string>& rowValues
) {
    return writeRowsToCsvFile(
        std::filesystem::path(outputFilename),
        CsvRow(headers.begin(), headers.end()),
        {CsvRow(rowValues.begin(), rowValues.end())}
    );
}
