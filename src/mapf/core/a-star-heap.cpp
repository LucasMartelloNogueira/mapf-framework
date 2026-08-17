#include "mapf/core/a-star-heap.hpp"

namespace mapf {

    bool CompareAStarNode::operator()(const AStarNode& v1, const AStarNode& v2) const {
        return v1.f > v2.f;
    }

}
