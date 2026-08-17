#include "mapf/pathfinding/sipp/edge_key.hpp"

namespace mapf {

    bool EdgeKey::operator==(const EdgeKey& other) const {
        return from == other.from && to == other.to;
    }

}
