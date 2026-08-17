#include "mapf/pathfinding/sipp/state_key.hpp"

namespace mapf {

    bool StateKey::operator==(const StateKey& other) const {
        return cell == other.cell && intervalIndex == other.intervalIndex;
    }

}
