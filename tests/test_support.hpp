#pragma once

#include <stdexcept>
#include <string>

inline void requireTest(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}
