#!/usr/bin/env bash

set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  scripts/profile_experiment.sh -- <mapf_app arguments>

Optional environment variables:
  PROFILE_OUTPUT_DIR       Artifact directory (must not already exist).
  FLAMEGRAPH_DIR           FlameGraph checkout (default: tools/FlameGraph).
  PERF_FREQUENCY           Sampling frequency in Hz (default: 199).
  PERF_STAT_REPETITIONS    Number of perf stat repetitions (default: 1).

The profiling executable must first be built with:
  cmake --preset profile
  cmake --build --preset profile --parallel
EOF
}

if [[ ${1:-} == "-h" || ${1:-} == "--help" ]]; then
    usage
    exit 0
fi

if [[ ${1:-} == "--" ]]; then
    shift
fi

if (( $# == 0 )); then
    echo "Error: provide the mapf_app experiment arguments after '--'." >&2
    usage >&2
    exit 2
fi

sampling_frequency=${PERF_FREQUENCY:-199}
stat_repetitions=${PERF_STAT_REPETITIONS:-1}

if [[ ! $sampling_frequency =~ ^[1-9][0-9]*$ ]]; then
    echo "Error: PERF_FREQUENCY must be a positive integer." >&2
    exit 2
fi

if [[ ! $stat_repetitions =~ ^[1-9][0-9]*$ ]]; then
    echo "Error: PERF_STAT_REPETITIONS must be a positive integer." >&2
    exit 2
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd -- "$script_dir/.." && pwd)
binary="$repository_root/build/profile/mapf_app"
flamegraph_dir=${FLAMEGRAPH_DIR:-$repository_root/tools/FlameGraph}
if [[ $flamegraph_dir != /* ]]; then
    flamegraph_dir="$repository_root/$flamegraph_dir"
fi
stack_collapse="$flamegraph_dir/stackcollapse-perf.pl"
flamegraph="$flamegraph_dir/flamegraph.pl"

if ! command -v perf >/dev/null 2>&1; then
    echo "Error: perf was not found in PATH. See docs/perf_flamegraph_profiling.md." >&2
    exit 2
fi

if [[ ! -x $binary ]]; then
    echo "Error: $binary does not exist or is not executable." >&2
    echo "Build it with: cmake --preset profile && cmake --build --preset profile --parallel" >&2
    exit 2
fi

if [[ ! -x $stack_collapse || ! -x $flamegraph ]]; then
    echo "Error: FlameGraph scripts were not found in $flamegraph_dir." >&2
    echo "Install them with: git clone https://github.com/brendangregg/FlameGraph.git tools/FlameGraph" >&2
    exit 2
fi

if [[ -n ${PROFILE_OUTPUT_DIR:-} ]]; then
    artifact_dir=$PROFILE_OUTPUT_DIR
    if [[ $artifact_dir != /* ]]; then
        artifact_dir="$repository_root/$artifact_dir"
    fi
else
    timestamp=$(date -u +%Y%m%dT%H%M%SZ)
    artifact_dir="$repository_root/profiling/${timestamp}-$$"
fi

if [[ -e $artifact_dir ]]; then
    echo "Error: profiling artifact directory already exists: $artifact_dir" >&2
    exit 2
fi

mkdir -p "$artifact_dir"

cd "$repository_root"

{
    printf 'created_utc=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    printf 'git_revision=%s\n' "$(git rev-parse HEAD 2>/dev/null || printf 'unknown')"
    printf 'build_type=RelWithDebInfo\n'
    printf 'binary=%s\n' "$binary"
    printf 'kernel=%s\n' "$(uname -r)"
    printf 'perf_version=%s\n' "$(perf version)"
    printf 'flamegraph_revision=%s\n' "$(git -C "$flamegraph_dir" rev-parse HEAD 2>/dev/null || printf 'unknown')"
    if [[ -r /proc/sys/kernel/perf_event_paranoid ]]; then
        printf 'perf_event_paranoid=%s\n' "$(< /proc/sys/kernel/perf_event_paranoid)"
    fi
    printf 'sampling_frequency_hz=%s\n' "$sampling_frequency"
    printf 'perf_stat_repetitions=%s\n' "$stat_repetitions"
    printf 'command='
    printf '%q ' "$binary" "$@"
    printf '\n'
} > "$artifact_dir/metadata.txt"

echo "Collecting general counters with perf stat..."
set +e
perf stat \
    -r "$stat_repetitions" \
    -o "$artifact_dir/perf-stat.txt" \
    -- "$binary" "$@"
stat_status=$?

echo "Recording sampled call stacks with perf record..."
perf record \
    -F "$sampling_frequency" \
    -g \
    --call-graph fp \
    -o "$artifact_dir/perf.data" \
    -- "$binary" "$@"
record_status=$?
set -e

{
    printf 'perf_stat_exit_status=%s\n' "$stat_status"
    printf 'perf_record_exit_status=%s\n' "$record_status"
} >> "$artifact_dir/metadata.txt"

if [[ ! -s $artifact_dir/perf.data ]]; then
    echo "Error: perf record did not produce usable data." >&2
    echo "Inspect $artifact_dir/perf-stat.txt and the permission troubleshooting guide." >&2
    if (( record_status != 0 )); then
        exit "$record_status"
    fi
    exit 1
fi

echo "Generating textual reports..."
perf report \
    -i "$artifact_dir/perf.data" \
    --stdio \
    --sort=overhead,symbol,dso \
    > "$artifact_dir/perf-report.txt"

perf report \
    -i "$artifact_dir/perf.data" \
    --stdio \
    --no-children \
    --sort=overhead,symbol,dso \
    > "$artifact_dir/perf-self-report.txt"

echo "Generating FlameGraph inputs and SVG..."
perf script \
    -i "$artifact_dir/perf.data" \
    > "$artifact_dir/perf.stacks"

"$stack_collapse" \
    "$artifact_dir/perf.stacks" \
    > "$artifact_dir/perf.folded"

"$flamegraph" \
    --title "MAPF CPU Flame Graph" \
    --countname samples \
    "$artifact_dir/perf.folded" \
    > "$artifact_dir/flamegraph.svg"

echo "Profiling artifacts generated in: $artifact_dir"

if (( record_status != 0 )); then
    exit "$record_status"
fi
exit "$stat_status"
