Created: 2026-09-09T20:43:54-03:00  
Author: Lucas Martello Nogueira  
Last updated: 2026-09-09T20:59:00-03:00  
AI model: GPT-5

# Perf and FlameGraph Profiling Specification

## Source plan

`specs/006-perf-flamegraph-profiling/plan.md`

## Objective

Provide two explicit and independent MAPF experiment workflows:

1. a normal optimized build and run workflow;
2. a symbolized optimized build and run workflow intended for Linux `perf` and FlameGraph profiling.

The workflow must be directly usable with the existing `mapf_app` CLI and benchmark files.

## Functional requirements

### Build configurations

- A CMake configure preset named `normal` must generate a `Release` build in `build/normal`.
- A CMake configure preset named `profile` must generate a `RelWithDebInfo` build in `build/profile`.
- Matching build presets named `normal` and `profile` must build their respective configurations.
- Project C++ compilation in `RelWithDebInfo` must preserve frame pointers when the compiler is GNU or Clang compatible.
- Normal and profiling configurations must not reuse the same CMake cache or overwrite each other's executables.

### Experiment runner

- `scripts/run_experiment.sh MODE -- ARGS...` must support exactly `normal` and `profile` modes.
- It must run the matching `mapf_app` executable from the repository root so repository-relative benchmark paths work.
- It must pass experiment arguments unchanged and preserve the executable's exit status.
- It must provide actionable messages for invalid mode, missing arguments, and missing executables.

### Profiling artifact generator

- `scripts/profile_experiment.sh -- ARGS...` must profile `build/profile/mapf_app`.
- It must check for `perf`, `tools/FlameGraph/stackcollapse-perf.pl`, `tools/FlameGraph/flamegraph.pl`, the profiling binary, and at least one experiment argument before running.
- It must accept the environment overrides `PROFILE_OUTPUT_DIR`, `FLAMEGRAPH_DIR`, `PERF_FREQUENCY`, and `PERF_STAT_REPETITIONS`.
- By default it must create a unique directory under `profiling/`.
- It must collect or generate:
  - `metadata.txt`;
  - `perf-stat.txt`;
  - `perf.data`;
  - `perf-report.txt`;
  - `perf-self-report.txt`;
  - `perf.stacks`;
  - `perf.folded`;
  - `flamegraph.svg`.
- It must use frame-pointer call-graph collection and a default sample frequency of 199 Hz.
- It must generate derived artifacts after an experiment failure when the recorded data remains valid, then return the experiment/recording status.
- It must not overwrite an explicitly selected artifact directory.

### Documentation

- The root README must show the separate normal and profiling build/run commands and link to the complete guide.
- The complete Portuguese guide must use actual project executable names, benchmark paths, solver flags, and output paths.
- It must explain why `RelWithDebInfo` is used for profiling even though the workflow is colloquially called a debug build.
- It must document dependency installation, FlameGraph setup, both build modes, direct and automated profiling, every generated artifact, interactive `perf report`, `perf annotate`, opening the SVG, permission failures, incomplete symbols/stacks, multithreaded experiments, repeated measurements, and before/after comparisons.
- Generated profiling data and the locally installed FlameGraph repository must be ignored by Git.

### Automated tests

- CTest must exercise both script help paths and their invalid mode, missing argument, and invalid numeric environment-value handling when Bash is available.

## Non-functional requirements

- No new C++ runtime library dependency may be introduced.
- Existing experiment CLI behavior and result artifacts must remain unchanged.
- Shell scripts must use strict error handling where it does not prevent artifact recovery and must quote filesystem paths and forwarded arguments.
- Compiler-specific flags must not be passed to unsupported compilers.

## Validation criteria

1. `cmake --list-presets` exposes the normal and profile presets.
2. Both presets configure and build successfully.
3. Profile compilation commands contain `-O2`, debug information, and `-fno-omit-frame-pointer` on GNU/Clang.
4. The complete CTest suite passes from the normal build.
5. Both run modes successfully invoke a representative repository experiment.
6. Both scripts pass Bash syntax validation, and their interface validation test passes through CTest.
7. If host permissions and dependencies permit sampling, the profiling script produces all specified artifacts; otherwise its diagnostic identifies the missing system prerequisite without invalidating build/test verification.
