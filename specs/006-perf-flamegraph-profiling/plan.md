Created: 2026-09-09T20:43:54-03:00  
Author: Lucas Martello Nogueira  
Last updated: 2026-09-09T20:59:00-03:00  
AI model: GPT-5

# Perf and FlameGraph Profiling Plan

## Prompt

use o arquivo tutorial_perf_flamegraph_mapf.md como referência para fazer as alterações necessárias para usar o perf e flameGraph para fazer o profiling e o flame graph de um build de debug de um experimento. Faça de maneira para que haja dois comandos de build e run: para para experimentos normais e outro para debug para o profiling. Faça a documentação de como fazer todo o processo de compilação do build de debug, execução e como gerar todos os artefatos de profiling e flamegraph

## Objective

Add a reproducible Linux CPU-profiling workflow for MAPF experiments while keeping normal experiment builds and profiling builds isolated. The profiling build will use `RelWithDebInfo`, rather than unoptimized `Debug`, so measurements retain compiler optimizations while exposing symbols, and it will preserve frame pointers for reliable `perf` call graphs.

## Files to create or change

- `CMakeLists.txt`: apply frame-pointer preservation to project code in `RelWithDebInfo` builds on supported compilers.
- `CMakePresets.json`: define separate normal (`Release`) and profiling (`RelWithDebInfo`) configure/build presets and build directories.
- `scripts/run_experiment.sh`: run `mapf_app` from either the normal or profiling build with the unchanged experiment CLI.
- `scripts/profile_experiment.sh`: collect `perf stat`, `perf record`, accumulated/self reports, raw and folded stacks, metadata, and the final FlameGraph SVG in one timestamped directory.
- `tests/scripts/profiling_scripts_test.sh`: verify help and invalid-input behavior through CTest.
- `docs/perf_flamegraph_profiling.md`: document prerequisites, both build/run workflows, artifact generation, analysis, troubleshooting, and reproducibility considerations using real repository paths and CLI arguments.
- `readme.md`: expose the two build/run workflows and link to the complete guide.
- `.gitignore`: ignore generated profiling data and the locally cloned FlameGraph toolkit.

## Libraries and tools

- CMake presets: built into the project's existing CMake toolchain and suitable for named, isolated build configurations.
- Linux `perf`: records hardware/software performance counters, sampled call stacks, reports, and annotated source/assembly.
- Brendan Gregg's FlameGraph scripts: the standard Perl tools used to collapse `perf script` stacks and render an interactive SVG.
- POSIX/Linux shell utilities: orchestrate the existing executable and profiling tools without adding a runtime dependency to the C++ project.

## Implementation outline

1. Configure `RelWithDebInfo` project targets with `-fno-omit-frame-pointer` for GNU and Clang C++ compilers.
2. Add `normal` and `profile` CMake presets using `build/normal` and `build/profile` respectively.
3. Add a run helper that maps the selected mode to the correct `mapf_app` binary and forwards all CLI arguments verbatim.
4. Add a profiling helper that validates dependencies, records a single experiment, produces every derived artifact, and reports the program exit status without discarding collected data.
5. Document installation, build, smoke-run, collection, interactive inspection, artifact meanings, permissions, symbol issues, and comparison workflow.
6. Add automated CLI validation for both shell helpers.
7. Validate JSON and shell syntax, configure/build both presets, run the automated test suite, run representative smoke experiments, and verify the profiling compile flags.

## Main command examples

```bash
cmake --preset normal
cmake --build --preset normal
./scripts/run_experiment.sh normal -- <mapf_app arguments>
```

```bash
cmake --preset profile
cmake --build --preset profile
./scripts/profile_experiment.sh -- <mapf_app arguments>
```

## Error handling and validation

- Scripts will reject unknown modes and missing experiment arguments.
- The normal run helper will report a missing build with the exact preset command needed to create it.
- The profiling helper will check `perf`, both FlameGraph scripts, and the profiling executable before collection.
- Profiling output will use a unique directory and the helper will refuse an explicitly supplied output directory that already exists.
- Derived reports will still be generated when the experiment itself returns a nonzero algorithmic status, provided `perf` produced usable sample data.

## TODO suggestions

- TODO: Add CI syntax validation for project shell scripts if a CI workflow is introduced.
- TODO: Consider adding differential flame graphs after a stable before/after performance benchmark policy is established.
