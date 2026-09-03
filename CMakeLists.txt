cmake_minimum_required(VERSION 3.20)

project(mapf_framework LANGUAGES CXX)

find_package(Threads REQUIRED)
include(CTest)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(mapf
    src/mapf/utils.cpp
    src/mapf/core/a-star-heap.cpp
    src/mapf/core/cell.cpp
    src/mapf/core/grid.cpp
    src/mapf/core/instance.cpp
    src/mapf/core/result.cpp
    src/mapf/pathfinding/a_star.cpp
    src/mapf/pathfinding/a_star_sipp.cpp
    src/mapf/pathfinding/sipp/compare_sipp_node.cpp
    src/mapf/pathfinding/sipp/edge_key.cpp
    src/mapf/pathfinding/sipp/edge_key_hash.cpp
    src/mapf/pathfinding/sipp/state_key.cpp
    src/mapf/pathfinding/sipp/state_key_hash.cpp
    src/mapf/solvers/local_path_repair_parallel_solver.cpp
    src/mapf/solvers/priority_planning_solver.cpp
)

target_include_directories(mapf
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(mapf
    PUBLIC
        Threads::Threads
)

add_executable(mapf_app
    src/main.cpp
)

target_link_libraries(mapf_app
    PRIVATE
        mapf
)

add_library(mapf_experiment_utils
    src/mapf/experiments/experiment_utils.cpp
)

target_link_libraries(mapf_experiment_utils
    PUBLIC
        mapf
)

target_include_directories(mapf_experiment_utils
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/mapf/experiments
)

target_compile_definitions(mapf_experiment_utils
    PUBLIC
        MAPF_REPOSITORY_ROOT="${CMAKE_CURRENT_SOURCE_DIR}"
)

add_executable(manual_experiment
    src/mapf/experiments/manual_experiment.cpp
)

target_link_libraries(manual_experiment
    PRIVATE
        mapf_experiment_utils
)

add_executable(benchmark_experiment
    src/mapf/experiments/benchmark_experiment.cpp
)

target_link_libraries(benchmark_experiment
    PRIVATE
        mapf_experiment_utils
)

if(BUILD_TESTING)
    add_executable(a_star_sipp_test
        tests/pathfinding/a_star_sipp_test.cpp
    )
    target_link_libraries(a_star_sipp_test PRIVATE mapf)
    target_include_directories(a_star_sipp_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    add_test(NAME a_star_sipp_test COMMAND a_star_sipp_test)

    add_executable(utils_collision_test
        tests/utils_collision_test.cpp
    )
    target_link_libraries(utils_collision_test PRIVATE mapf)
    target_include_directories(utils_collision_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    add_test(NAME utils_collision_test COMMAND utils_collision_test)

    add_executable(local_path_repair_parallel_solver_test
        tests/solvers/local_path_repair_parallel_solver_test.cpp
    )
    target_link_libraries(local_path_repair_parallel_solver_test PRIVATE mapf)
    target_include_directories(local_path_repair_parallel_solver_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    add_test(
        NAME local_path_repair_parallel_solver_test
        COMMAND local_path_repair_parallel_solver_test
    )
endif()
