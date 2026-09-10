#!/usr/bin/env bash

set -euo pipefail

repository_root=${1:?repository root argument is required}
run_script="$repository_root/scripts/run_experiment.sh"
profile_script="$repository_root/scripts/profile_experiment.sh"

expect_status() {
    local expected_status=$1
    shift

    set +e
    "$@" >/dev/null 2>&1
    local actual_status=$?
    set -e

    if (( actual_status != expected_status )); then
        echo "Expected status $expected_status, got $actual_status from: $*" >&2
        return 1
    fi
}

"$run_script" --help >/dev/null
"$profile_script" --help >/dev/null

expect_status 2 "$run_script" invalid -- -map ignored
expect_status 2 "$run_script" normal
expect_status 2 "$profile_script"
expect_status 2 env PERF_FREQUENCY=0 "$profile_script" -- -map ignored
expect_status 2 env PERF_STAT_REPETITIONS=invalid "$profile_script" -- -map ignored
