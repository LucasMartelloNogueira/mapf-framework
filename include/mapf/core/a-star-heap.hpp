#pragma once

#include <vector>
#include <queue>

#include "a-star-node.hpp"

namespace mapf {

    struct CompareAStarNode {
        bool operator()(const AStarNode& v1, const AStarNode& v2) {
            return v1.f > v2.f;
        }
    };

    using AStarHeap = std::priority_queue<AStarNode, std::vector<AStarNode>, CompareAStarNode>;
}