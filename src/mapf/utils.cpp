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
    startOwners.reserve(paths.size());
    goalVertexLookup.reserve(paths.size());
    std::size_t explicitCells = 0;
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
            for (mapf::Cell* cell : path) {
                if (cell == nullptr) {
                    throw std::invalid_argument("Paths must not contain null cells.");
                }
            }
            const int arrival = static_cast<int>(path.size() - 1);
            if (!startOwners.emplace(path.front(), i).second ||
                !goalVertexLookup.emplace(path.back(), GoalOccupancy{i, arrival}).second) {
                throw std::invalid_argument("Path starts and path goals must each be unique.");
            }
        }
        ++i;
    }

    struct Cursor {
        int agent;
        std::list<mapf::Cell*>::const_iterator current;
        std::list<mapf::Cell*>::const_iterator end;
        mapf::Cell* previous = nullptr;
    };
    struct Visit { mapf::Cell* cell; int agent; };
    struct Movement { mapf::EdgeTime key; int agent; bool forward; };
    std::vector<Cursor> cursors;
    std::vector<Visit> visits;
    std::vector<Movement> movements;
    for (std::size_t index = 0; index < paths.size(); ++index) {
        if (!paths[index].empty()) {
            cursors.push_back({static_cast<int>(index), paths[index].begin(), paths[index].end()});
        }
    }
    visits.reserve(cursors.size());
    movements.reserve(cursors.size());

    // Keep occupancy for one time step only. Hash-table nodes and participant
    // sets are allocated exclusively for actual conflicts, not every visit.
    for (std::size_t tick = 0; !cursors.empty(); ++tick) {
        const int time = static_cast<int>(tick);
        visits.clear();
        movements.clear();
        for (std::size_t index = 0; index < cursors.size();) {
            auto& cursor = cursors[index];
            mapf::Cell* cell = *cursor.current;
            visits.push_back({cell, cursor.agent});
            if (cursor.previous != nullptr && cursor.previous != cell) {
                const auto key = mapf::makeEdgeTime(cursor.previous, cell, time);
                movements.push_back({key, cursor.agent, cursor.previous == key.first});
            }
            cursor.previous = cell;
            if (++cursor.current == cursor.end) {
                cursor = cursors.back();
                cursors.pop_back();
            } else {
                ++index;
            }
        }
        std::sort(visits.begin(), visits.end(), [](const Visit& a, const Visit& b) {
            return mapf::cellComesBefore(a.cell, b.cell);
        });
        for (std::size_t begin = 0; begin < visits.size();) {
            std::size_t end = begin + 1;
            mapf::Cell* cell = visits[begin].cell;
            while (end < visits.size() && visits[end].cell == cell) ++end;
            const auto goal = goalVertexLookup.find(cell);
            const bool parked = goal != goalVertexLookup.end() && goal->second.arrivalTime < time;
            if (end - begin > 1 || parked) {
                auto& event = conflicts.vertexEvents[{cell, time}];
                for (std::size_t j = begin; j < end; ++j) {
                    event.participants.insert(visits[j].agent);
                    conflicts.byAgent[visits[j].agent].push_back(mapf::CellConflict{cell, time});
                }
                if (parked) event.participants.insert(goal->second.i);
            }
            begin = end;
        }
        std::sort(movements.begin(), movements.end(), [](const Movement& a, const Movement& b) {
            if (a.key.first != b.key.first) return mapf::cellComesBefore(a.key.first, b.key.first);
            return mapf::cellComesBefore(a.key.second, b.key.second);
        });
        for (std::size_t begin = 0; begin < movements.size();) {
            std::size_t end = begin + 1;
            bool forward = movements[begin].forward;
            bool reverse = !forward;
            while (end < movements.size() && movements[end].key == movements[begin].key) {
                forward = forward || movements[end].forward;
                reverse = reverse || !movements[end].forward;
                ++end;
            }
            if (forward && reverse) {
                const auto& key = movements[begin].key;
                auto& event = conflicts.edgeEvents[key];
                const mapf::EdgeConflict record {key.first, key.second, time};
                for (std::size_t j = begin; j < end; ++j) {
                    auto& participants = movements[j].forward ? event.forward : event.reverse;
                    participants.insert(movements[j].agent);
                    conflicts.byAgent[movements[j].agent].push_back(record);
                }
            }
            begin = end;
        }
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

void UpdateSolutionConflictsV2(
    const std::list<mapf::Cell*>& newPath,
    const mapf::PathReservationState& state,
    mapf::SolutionConflicts& conflicts,
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
        return;
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
    return;
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
