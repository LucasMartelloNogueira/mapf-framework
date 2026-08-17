#pragma once

#include <cstddef>

#include "edge_key.hpp"

namespace mapf {

    struct EdgeKeyHash {
        std::size_t operator()(const EdgeKey& key) const;
    };

}
