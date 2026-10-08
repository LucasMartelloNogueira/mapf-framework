#pragma once

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <variant>

#include "conflict_key.hpp"

namespace mapf {

    struct VertexEvent {
        std::unordered_set<int> participants;
    };

    struct EdgeEvent {
        std::unordered_set<int> forward;
        std::unordered_set<int> reverse;

        bool active() const {
            return !forward.empty() && !reverse.empty();
        }
    };

    // Participant identities are path indexes; cell pointers are non-owning.
    struct SolutionConflicts {
        std::unordered_map<CellTime, VertexEvent, CellTimeHash> vertexEvents; // conflitos de vertice
        std::unordered_map<EdgeTime, EdgeEvent, EdgeTimeHash> edgeEvents; // conflitos de aresta
        std::vector<std::vector<std::variant<CellConflict, EdgeConflict>>> byAgent; // conflitos por agente

        bool empty() const {
            return vertexEvents.empty() && edgeEvents.empty();
        }
    };

}
