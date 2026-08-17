#pragma once

#include "sipp_node.hpp"

namespace mapf {

    struct CompareSippNode {
        bool operator()(const SippNode& a, const SippNode& b) const;
    };

}
