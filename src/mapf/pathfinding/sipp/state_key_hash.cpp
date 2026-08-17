#include "mapf/pathfinding/sipp/state_key_hash.hpp"

#include <functional>

namespace mapf {

    std::size_t StateKeyHash::operator()(const StateKey& key) const {
        std::size_t cellHash = std::hash<Cell*>{}(key.cell);
        std::size_t intervalHash = std::hash<int>{}(key.intervalIndex);
        return cellHash ^ (intervalHash << 1);
    }

}
