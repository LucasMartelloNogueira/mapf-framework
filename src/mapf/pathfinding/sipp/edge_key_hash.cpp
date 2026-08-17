#include "mapf/pathfinding/sipp/edge_key_hash.hpp"

#include <functional>

namespace mapf {

    std::size_t EdgeKeyHash::operator()(const EdgeKey& key) const {
        std::size_t fromHash = std::hash<Cell*>{}(key.from);
        std::size_t toHash = std::hash<Cell*>{}(key.to);
        return fromHash ^ (toHash << 1);
    }

}
