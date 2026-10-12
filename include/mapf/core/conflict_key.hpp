#pragma once

#include <cstddef>
#include <variant>

#include "cell_conflict.hpp"
#include "edge_conflict.hpp"

namespace mapf {
    struct CellTime {
        Cell* cell;
        int time;
        bool operator==(const CellTime& other) const;
    };

    struct EdgeTime {
        Cell* first;
        Cell* second;
        int time;
        bool operator==(const EdgeTime& other) const;
    };

    struct CellTimeHash {
        std::size_t operator()(const CellTime& key) const;
    };

    struct EdgeTimeHash {
        std::size_t operator()(const EdgeTime& key) const;
    };

    bool cellComesBefore(const Cell* first, const Cell* second);
    EdgeTime makeEdgeTime(Cell* from, Cell* to, int arrivalTime);
    int conflictTime(const std::variant<CellConflict, EdgeConflict>& conflict);
    bool conflictRecordComesBefore(
        const std::variant<CellConflict, EdgeConflict>& first,
        const std::variant<CellConflict, EdgeConflict>& second
    );
}
