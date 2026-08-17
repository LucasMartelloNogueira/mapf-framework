#include "mapf/pathfinding/sipp/compare_sipp_node.hpp"

namespace mapf {

    bool CompareSippNode::operator()(const SippNode& a, const SippNode& b) const {
        if (a.f != b.f) {
            return a.f > b.f;
        }

        return a.g > b.g;
    }

}
