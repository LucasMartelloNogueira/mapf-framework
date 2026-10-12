#include "mapf/core/conflict_key.hpp"

#include <functional>
#include <tuple>
#include <utility>

namespace {
    std::size_t combineHash(std::size_t seed, std::size_t value) {
        return seed ^ (value + static_cast<std::size_t>(0x9e3779b9U) +
            (seed << 6) + (seed >> 2));
    }

    std::tuple<int, int, int, int, int, int> recordOrder(
        const std::variant<mapf::CellConflict, mapf::EdgeConflict>& conflict
    ) {
        const mapf::CellConflict* vertex = std::get_if<mapf::CellConflict>(&conflict);
        if (vertex != nullptr) {
            return {vertex->time, 0, vertex->cell->position.x, vertex->cell->position.y,
                vertex->cell->position.x, vertex->cell->position.y};
        }
        const mapf::EdgeConflict& edge = std::get<mapf::EdgeConflict>(conflict);
        return {edge.time, 1, edge.cell_1->position.x, edge.cell_1->position.y,
            edge.cell_2->position.x, edge.cell_2->position.y};
    }
}

namespace mapf {
    bool CellTime::operator==(const CellTime& other) const {
        return cell == other.cell && time == other.time;
    }

    bool EdgeTime::operator==(const EdgeTime& other) const {
        return first == other.first && second == other.second && time == other.time;
    }

    std::size_t CellTimeHash::operator()(const CellTime& key) const {
        return combineHash(std::hash<Cell*>{}(key.cell), std::hash<int>{}(key.time));
    }

    std::size_t EdgeTimeHash::operator()(const EdgeTime& key) const {
        const std::size_t seed = combineHash(
            std::hash<Cell*>{}(key.first), std::hash<Cell*>{}(key.second));
        return combineHash(seed, std::hash<int>{}(key.time));
    }

    bool cellComesBefore(const Cell* first, const Cell* second) {
        return std::tie(first->position.x, first->position.y) <
            std::tie(second->position.x, second->position.y);
    }

    EdgeTime makeEdgeTime(Cell* from, Cell* to, int arrivalTime) {
        if (cellComesBefore(to, from)) {
            std::swap(from, to);
        }
        return {from, to, arrivalTime};
    }

    int conflictTime(const std::variant<CellConflict, EdgeConflict>& conflict) {
        const CellConflict* vertex = std::get_if<CellConflict>(&conflict);
        return vertex != nullptr ? vertex->time : std::get<EdgeConflict>(conflict).time;
    }

    bool conflictRecordComesBefore(
        const std::variant<CellConflict, EdgeConflict>& first,
        const std::variant<CellConflict, EdgeConflict>& second
    ) {
        return recordOrder(first) < recordOrder(second);
    }
}
