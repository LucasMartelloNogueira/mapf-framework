#pragma once

#include <cstddef>

#include "state_key.hpp"

namespace mapf {

    struct StateKeyHash {
        std::size_t operator()(const StateKey& key) const;
    };

}
