#pragma once

#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "edge_key.hpp"
#include "edge_key_hash.hpp"
#include "interval.hpp"

namespace mapf {

    inline constexpr int SAFE_INTERVAL_INFINITY = std::numeric_limits<int>::max() / 4;

    using SafeIntervalsByCell = std::unordered_map<Cell*, std::vector<Interval>>;
    using BlockedEdgeArrivals = std::unordered_map<EdgeKey, std::unordered_set<int>, EdgeKeyHash>;

    struct SafeIntervalTable {
        SafeIntervalsByCell safeIntervalsByCell;
        BlockedEdgeArrivals blockedEdgeArrivals;
    };

}
