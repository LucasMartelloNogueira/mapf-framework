#!/usr/bin/env bash

set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  scripts/run_experiment.sh <normal|profile> -- <mapf_app arguments>

Examples:
  scripts/run_experiment.sh normal -- -map benchmarks/maps/empty-8-8.map \
    -scen benchmarks/scenarios/empty-8-8/random/empty-8-8-random-1.scen \
    -solver PriorityPlanningSolver -agents 3

  scripts/run_experiment.sh profile -- -map benchmarks/maps/empty-8-8.map \
    -scen benchmarks/scenarios/empty-8-8/random/empty-8-8-random-1.scen \
    -solver PriorityPlanningSolver -agents 3
EOF
}

if [[ ${1:-} == "-h" || ${1:-} == "--help" ]]; then
    usage
    exit 0
fi

mode=${1:-}
if [[ -z $mode ]]; then
    usage >&2
    exit 2
fi
shift

if [[ ${1:-} == "--" ]]; then
    shift
fi

if (( $# == 0 )); then
    echo "Error: provide the mapf_app experiment arguments after '--'." >&2
    usage >&2
    exit 2
fi

case "$mode" in
    normal|profile)
        ;;
    *)
        echo "Error: build mode must be 'normal' or 'profile', got '$mode'." >&2
        usage >&2
        exit 2
        ;;
esac

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd -- "$script_dir/.." && pwd)
binary="$repository_root/build/$mode/mapf_app"

if [[ ! -x $binary ]]; then
    echo "Error: $binary does not exist or is not executable." >&2
    echo "Build it with: cmake --preset $mode && cmake --build --preset $mode --parallel" >&2
    exit 2
fi

cd "$repository_root"
exec "$binary" "$@"
