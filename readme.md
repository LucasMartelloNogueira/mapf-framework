# Comandos

## para compilar

    g++ -std=c++20 -Iinclude \
      src/main.cpp \
      src/mapf/core/a-star-heap.cpp \
      src/mapf/core/cell.cpp \
      src/mapf/core/grid.cpp \
      src/mapf/pathfinding/a_star.cpp \
      src/mapf/pathfinding/a_star_sipp.cpp \
      src/mapf/pathfinding/sipp/compare_sipp_node.cpp \
      src/mapf/pathfinding/sipp/edge_key.cpp \
      src/mapf/pathfinding/sipp/edge_key_hash.cpp \
      src/mapf/pathfinding/sipp/state_key.cpp \
      src/mapf/pathfinding/sipp/state_key_hash.cpp \
      -o bin/main

## comando para rodar

    ./bin/main

## Build system com CMake

Configure o projeto:

    cmake -S . -B build

Compile o executavel:

    cmake --build build

Rode o executavel:

    ./build/mapf_app

Bibliotecas externas podem ser adicionadas no `CMakeLists.txt` com `find_package(...)` e vinculadas aos targets com `target_link_libraries(...)`.
