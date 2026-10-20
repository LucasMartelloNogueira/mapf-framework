#!/usr/bin/env bash

set -euo pipefail

start_seconds=$SECONDS

usage() {
    cat <<'EOF'
Uso:
  scripts/first_bad_max_new_conflict_value.sh [normal|profile]

Para cada mapa e quantidade de agentes configurados, executa o
LocalPathRepairIterativeSolver com -maxNewConflicts começando em 0 e aumentando
de um em um. A busca termina no primeiro valor k em que sumOfCosts, makespan,
injustice e durationSeconds são todos maiores que os valores da execução k-1.
Se sumOfCosts, makespan e injustice permanecerem iguais por 10 iterações
consecutivas, a busca também termina e registra k-10 como resultado.

O cenário usado para cada mapa é:
  benchmarks/scenarios/<mapa>/random/<mapa>-random-1.scen

O modo padrão é normal. Cada execução salva seus artefatos em:
  results/first_bad_max_new_conflict_value_<timestamp_seconds>/

Opções:
  -h, --help  Mostra esta ajuda.
EOF
}

mode=${1:-normal}
case "$mode" in
    normal|profile)
        if (( $# > 1 )); then
            echo "Erro: informe no máximo um modo de compilação." >&2
            usage >&2
            exit 2
        fi
        ;;
    -h|--help)
        usage
        exit 0
        ;;
    *)
        echo "Erro: argumento desconhecido: $mode" >&2
        usage >&2
        exit 2
        ;;
esac

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd -- "$script_dir/.." && pwd)
binary="$repository_root/build/$mode/mapf_app"

if [[ ! -x $binary ]]; then
    echo "Erro: $binary não existe ou não é executável." >&2
    echo "Compile com: cmake --preset $mode && cmake --build --preset $mode --parallel" >&2
    exit 2
fi

cd "$repository_root"
export LC_ALL=C

map_configs=(
    "den520d 333 666 1000"
    "empty-8-8 5 11 17"
    "empty-32-32 36 72 109"
    "ost003d 219 438 658"
    "random-32-32-20 14 28 42"
    "warehouse-10-20-10-2-1 49 98 148"
)

timestamp_seconds=$(date +%s)
output_folder_name="first_bad_max_new_conflict_value_${timestamp_seconds}"
output_root="$repository_root/results/$output_folder_name"
if [[ -e $output_root ]]; then
    echo "Erro: a pasta de saída já existe: $output_root" >&2
    exit 2
fi

declare -A first_bad_values
completed_runs=0
stagnation_limit=10
current_sum_of_costs=0
current_makespan=0
current_injustice=0
current_duration_seconds=0

elapsed_seconds() {
    printf '%d' "$((SECONDS - start_seconds))"
}

load_latest_metrics() {
    local expected_runs=$((completed_runs + 1))
    local run_directory
    local stats_file
    local metrics
    local -a run_directories=()

    if [[ -d $output_root ]]; then
        mapfile -t run_directories < <(
            find "$output_root" -mindepth 1 -maxdepth 1 -type d -printf '%p\n' | sort
        )
    fi

    if (( ${#run_directories[@]} != expected_runs )); then
        echo "Erro: esperava $expected_runs pastas de execução em $output_root, mas encontrei ${#run_directories[@]}." >&2
        exit 2
    fi

    run_directory=${run_directories[${#run_directories[@]}-1]}
    stats_file="$run_directory/${run_directory##*/}_stats.csv"
    if [[ ! -r $stats_file ]]; then
        echo "Erro: arquivo de estatísticas não encontrado: $stats_file" >&2
        exit 2
    fi

    metrics=$(awk -F',' '
        NR == 1 {
            for (i = 1; i <= NF; i++) {
                column[$i] = i
            }
            next
        }
        NR == 2 {
            print $(column["sumOfCosts"]) "\t" \
                $(column["makespan"]) "\t" \
                $(column["injustice"]) "\t" \
                $(column["durationSeconds"])
            exit
        }
    ' "$stats_file")

    IFS=$'\t' read -r \
        current_sum_of_costs \
        current_makespan \
        current_injustice \
        current_duration_seconds <<< "$metrics"

    for metric in \
        "$current_sum_of_costs" \
        "$current_makespan" \
        "$current_injustice" \
        "$current_duration_seconds"; do
        if [[ ! $metric =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
            echo "Erro: métrica inválida em $stats_file: '$metric'" >&2
            exit 2
        fi
    done

    completed_runs=$expected_runs
}

run_experiment() {
    local map_name=$1
    local agents=$2
    local max_new_conflicts=$3
    local map_path="benchmarks/maps/$map_name.map"
    local scenario_path="benchmarks/scenarios/$map_name/random/$map_name-random-1.scen"
    local exit_code=0
    local command_output
    local -a command=(
        "$binary"
        -map "$map_path"
        -scen "$scenario_path"
        -solver LocalPathRepairIterativeSolver
        -agents "$agents"
        -maxNewConflicts "$max_new_conflicts"
        -outputFolder "$output_folder_name"
        -saveResults true
    )

    printf '[%6ss] mapa=%-25s agentes=%-4d maxNewConflicts=%-4d ... ' \
        "$(elapsed_seconds)" "$map_name" "$agents" "$max_new_conflicts"

    if command_output=$("${command[@]}" 2>&1); then
        exit_code=0
    else
        exit_code=$?
    fi

    if (( exit_code > 1 )); then
        printf 'erro (código %d)\n' "$exit_code"
        if [[ -n $command_output ]]; then
            printf '%s\n' "$command_output" >&2
        fi
        exit "$exit_code"
    fi

    load_latest_metrics
    printf 'retorno=%d; sumOfCosts=%s; makespan=%s; injustice=%s; durationSeconds=%s\n' \
        "$exit_code" \
        "$current_sum_of_costs" \
        "$current_makespan" \
        "$current_injustice" \
        "$current_duration_seconds"
}

all_metrics_are_worse() {
    local previous_sum_of_costs=$1
    local previous_makespan=$2
    local previous_injustice=$3
    local previous_duration_seconds=$4
    local comparison

    if ! comparison=$(awk \
        -v current_sum="$current_sum_of_costs" \
        -v previous_sum="$previous_sum_of_costs" \
        -v current_makespan="$current_makespan" \
        -v previous_makespan="$previous_makespan" \
        -v current_injustice="$current_injustice" \
        -v previous_injustice="$previous_injustice" \
        -v current_duration="$current_duration_seconds" \
        -v previous_duration="$previous_duration_seconds" '
            BEGIN {
                all_worse = current_sum > previous_sum && current_makespan > previous_makespan && current_injustice > previous_injustice && current_duration > previous_duration
                print all_worse ? "true" : "false"
            }
        '); then
        echo "Erro: não foi possível comparar as métricas." >&2
        exit 2
    fi

    [[ $comparison == true ]]
}

find_first_bad_value() {
    local map_name=$1
    local agents=$2
    local max_new_conflicts=0
    local previous_sum_of_costs
    local previous_makespan
    local previous_injustice
    local previous_duration_seconds
    local unchanged_iterations=0
    local stagnation_value

    printf '\n[%6ss] iniciando mapa=%s; agentes=%d\n' \
        "$(elapsed_seconds)" "$map_name" "$agents"

    run_experiment "$map_name" "$agents" "$max_new_conflicts"
    previous_sum_of_costs=$current_sum_of_costs
    previous_makespan=$current_makespan
    previous_injustice=$current_injustice
    previous_duration_seconds=$current_duration_seconds

    while true; do
        max_new_conflicts=$((max_new_conflicts + 1))
        run_experiment "$map_name" "$agents" "$max_new_conflicts"

        if all_metrics_are_worse \
            "$previous_sum_of_costs" \
            "$previous_makespan" \
            "$previous_injustice" \
            "$previous_duration_seconds"; then
            first_bad_values["$map_name|$agents"]=$max_new_conflicts
            printf '[%6ss] primeiro parâmetro ruim: mapa=%s; agentes=%d; maxNewConflicts=%d\n' \
                "$(elapsed_seconds)" "$map_name" "$agents" "$max_new_conflicts"
            return
        fi

        if [[
            $current_sum_of_costs == "$previous_sum_of_costs" &&
            $current_makespan == "$previous_makespan" &&
            $current_injustice == "$previous_injustice"
        ]]; then
            unchanged_iterations=$((unchanged_iterations + 1))
            printf '[%6ss] métricas sem mudança: mapa=%s; agentes=%d; sequência=%d/%d\n' \
                "$(elapsed_seconds)" "$map_name" "$agents" \
                "$unchanged_iterations" "$stagnation_limit"
        else
            unchanged_iterations=0
        fi

        if (( unchanged_iterations == stagnation_limit )); then
            stagnation_value=$((max_new_conflicts - stagnation_limit))
            first_bad_values["$map_name|$agents"]=$stagnation_value
            printf '[%6ss] encerrado por estagnação: mapa=%s; agentes=%d; k=%d; valor registrado=%d (k-%d)\n' \
                "$(elapsed_seconds)" "$map_name" "$agents" "$max_new_conflicts" \
                "$stagnation_value" "$stagnation_limit"
            return
        fi

        previous_sum_of_costs=$current_sum_of_costs
        previous_makespan=$current_makespan
        previous_injustice=$current_injustice
        previous_duration_seconds=$current_duration_seconds
    done
}

for config in "${map_configs[@]}"; do
    read -r map_name first_agents second_agents third_agents <<< "$config"
    map_path="benchmarks/maps/$map_name.map"
    scenario_path="benchmarks/scenarios/$map_name/random/$map_name-random-1.scen"

    for input_file in "$map_path" "$scenario_path"; do
        if [[ ! -r $input_file ]]; then
            echo "Erro: arquivo de entrada indisponível: $input_file" >&2
            exit 2
        fi
    done

    available_agents=$(awk 'NR > 1 && NF { count++ } END { print count + 0 }' "$scenario_path")
    if (( third_agents > available_agents )); then
        echo "Erro: $scenario_path contém $available_agents agentes, menos que os $third_agents solicitados." >&2
        exit 2
    fi
done

trap 'printf "\nBusca interrompida após %d segundos. Resultados parciais: %s\n" "$((SECONDS - start_seconds))" "$output_root" >&2; exit 130' INT
trap 'printf "\nBusca interrompida após %d segundos. Resultados parciais: %s\n" "$((SECONDS - start_seconds))" "$output_root" >&2; exit 143' TERM

printf 'Iniciando busca do primeiro maxNewConflicts ruim.\n'
printf 'Build: %s; solver: LocalPathRepairIterativeSolver.\n' "$mode"
printf 'Resultados: %s\n' "$output_root"

for config in "${map_configs[@]}"; do
    read -r map_name first_agents second_agents third_agents <<< "$config"
    for agents in "$first_agents" "$second_agents" "$third_agents"; do
        find_first_bad_value "$map_name" "$agents"
    done
done

map_column_width=25
value_column_width=25
printf '\nTabela final — primeiro maxNewConflicts ruim\n\n'
printf "%-${map_column_width}s" "Mapa"
printf " | %-${value_column_width}s" "1ª quantidade de agentes"
printf " | %-${value_column_width}s" "2ª quantidade de agentes"
printf " | %-${value_column_width}s\n" "3ª quantidade de agentes"

separator_width=$((map_column_width + (3 * (value_column_width + 3))))
printf '%*s\n' "$separator_width" '' | tr ' ' '-'

for config in "${map_configs[@]}"; do
    read -r map_name first_agents second_agents third_agents <<< "$config"
    printf "%-${map_column_width}s" "$map_name"
    for agents in "$first_agents" "$second_agents" "$third_agents"; do
        value=${first_bad_values["$map_name|$agents"]}
        printf " | %-${value_column_width}s" "$value ($agents agentes)"
    done
    printf '\n'
done

total_elapsed_seconds=$((SECONDS - start_seconds))
printf '\nResultados salvos em: %s\n' "$output_root"
printf 'Tempo total: %d segundos.\n' "$total_elapsed_seconds"
