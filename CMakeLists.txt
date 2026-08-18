cmake_minimum_required(VERSION 3.20)

project(mapf_framework LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(mapf
    src/mapf/utils.cpp
    src/mapf/core/a-star-heap.cpp
    src/mapf/core/cell.cpp
    src/mapf/core/grid.cpp
    src/mapf/pathfinding/a_star.cpp
    src/mapf/pathfinding/a_star_sipp.cpp
    src/mapf/pathfinding/sipp/compare_sipp_node.cpp
    src/mapf/pathfinding/sipp/edge_key.cpp
    src/mapf/pathfinding/sipp/edge_key_hash.cpp
    src/mapf/pathfinding/sipp/state_key.cpp
    src/mapf/pathfinding/sipp/state_key_hash.cpp
)

target_include_directories(mapf
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add external libraries with find_package(...) and target_link_libraries(mapf ...).

add_executable(mapf_app
    src/main.cpp
)

target_link_libraries(mapf_app
    PRIVATE
        mapf
)
