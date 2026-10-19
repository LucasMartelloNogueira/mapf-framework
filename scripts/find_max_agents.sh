#!/usr/bin/env bash

set -euo pipefail

usage() {
    cat <<'EOF'
Uso:
  scripts/find_max_agents.sh [normal|profile]

Descobre, para cada combinação de mapa e solver, o maior número k de agentes
resolvido pelo mapf_app. A busca dobra a quantidade de agentes até a primeira
falha e depois executa uma busca binária entre o último sucesso e a falha.

O cenário usado para cada mapa é:
  benchmarks/scenarios/<mapa>/random/<mapa>-random-1.scen

O modo padrão é normal. Os experimentos usam -saveResults false.

Variáveis de ambiente opcionais:
  PARALLEL_THREADS  Threads do LocalPathRepairParallelSolver (padrão: 8).

Opções:
  -h, --help        Mostra esta ajuda.
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

parallel_threads=${PARALLEL_THREADS:-8}
if [[ ! $parallel_threads =~ ^[1-9][0-9]*$ ]]; then
    echo "Erro: PARALLEL_THREADS deve ser um inteiro maior que zero." >&2
    exit 2
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd -- "$script_dir/.." && pwd)
binary="$repository_root/build/$mode/mapf_app"

if [[ ! -x $binary ]]; then
    echo "Erro: $binary não existe ou não é executável." >&2
    echo "Compile com: cmake --preset $mode && cmake --build --preset $mode --parallel" >&2
    exit 2
fi

cd "$repository_root"

maps=(
    den520d
    empty-8-8
    empty-32-32
    ost003d
    random-32-32-20
    warehouse-10-20-10-2-1
)

solvers=(
    PriorityPlanningSolver
    LocalPathRepairParallelSolver
    LocalPathRepairIterativeSolver
    FullPathRepairIterativeSolver
)

declare -A maximum_agents
start_seconds=$SECONDS

elapsed_seconds() {
    printf '%d' "$((SECONDS - start_seconds))"
}

run_trial() {
    local map_path=$1
    local scenario_path=$2
    local solver=$3
    local agents=$4
    local phase=$5
    local exit_code=0
    local -a command=(
        "$binary"
        -map "$map_path"
        -scen "$scenario_path"
        -solver "$solver"
        -agents "$agents"
        -saveResults false
    )

    if [[ $solver == LocalPathRepairParallelSolver ]]; then
        command+=(-threads "$parallel_threads")
    fi

    printf '[%6ss] %-14s mapa=%-25s solver=%-32s agentes=%d ... ' \
        "$(elapsed_seconds)" "$phase" "${map_path##*/}" "$solver" "$agents"

    if "${command[@]}" >/dev/null 2>&1; then
        printf 'sucesso\n'
        return 0
    else
        exit_code=$?
        printf 'falha (código %d)\n' "$exit_code"
        return 1
    fi
}

find_max_agents() {
    local map_name=$1
    local solver=$2
    local scenario_agents=$3
    local map_path="benchmarks/maps/$map_name.map"
    local scenario_path="benchmarks/scenarios/$map_name/random/$map_name-random-1.scen"
    local last_success=0
    local first_failure=1
    local candidate=1
    local middle=0

    while true; do
        if run_trial "$map_path" "$scenario_path" "$solver" "$candidate" "expansão"; then
            last_success=$candidate
            candidate=$((candidate * 2))
        else
            first_failure=$candidate
            break
        fi
    done

    while (( first_failure - last_success > 1 )); do
        middle=$(((last_success + first_failure) / 2))
        if run_trial "$map_path" "$scenario_path" "$solver" "$middle" "busca binária"; then
            last_success=$middle
        else
            first_failure=$middle
        fi
    done

    maximum_agents["$map_name|$solver"]=$last_success
    printf '[%6ss] resultado      mapa=%-25s solver=%-32s k=%d; k+1=%d falhou\n' \
        "$(elapsed_seconds)" "$map_name" "$solver" "$last_success" "$first_failure"
    if (( last_success == scenario_agents )); then
        printf '[%6ss] aviso          mapa=%-25s limite de %d agentes do cenário atingido\n' \
            "$(elapsed_seconds)" "$map_name" "$scenario_agents"
    fi
}

for map_name in "${maps[@]}"; do
    map_path="benchmarks/maps/$map_name.map"
    scenario_path="benchmarks/scenarios/$map_name/random/$map_name-random-1.scen"
    for input_file in "$map_path" "$scenario_path"; do
        if [[ ! -r $input_file ]]; then
            echo "Erro: arquivo de entrada indisponível: $input_file" >&2
            exit 2
        fi
    done
done

trap 'printf "\nBusca interrompida após %d segundos.\n" "$((SECONDS - start_seconds))" >&2; exit 130' INT
trap 'printf "\nBusca interrompida após %d segundos.\n" "$((SECONDS - start_seconds))" >&2; exit 143' TERM

total_pairs=$((${#maps[@]} * ${#solvers[@]}))
current_pair=0
printf 'Iniciando busca de limites: %d mapas × %d solvers = %d combinações.\n' \
    "${#maps[@]}" "${#solvers[@]}" "$total_pairs"
printf 'Build: %s; threads do solver paralelo: %d; resultados em arquivos: desabilitados.\n' \
    "$mode" "$parallel_threads"

for map_name in "${maps[@]}"; do
    scenario_path="benchmarks/scenarios/$map_name/random/$map_name-random-1.scen"
    available_agents=$(awk 'NR > 1 && NF { count++ } END { print count + 0 }' "$scenario_path")

    for solver in "${solvers[@]}"; do
        current_pair=$((current_pair + 1))
        printf '\n[%6ss] combinação %d/%d: mapa=%s; solver=%s; agentes no cenário=%d\n' \
            "$(elapsed_seconds)" "$current_pair" "$total_pairs" "$map_name" "$solver" \
            "$available_agents"
        find_max_agents "$map_name" "$solver" "$available_agents"
    done
done

map_column_width=25
solver_column_width=34
printf '\nTabela final — máximo de agentes resolvidos\n\n'
printf "%-${map_column_width}s" "Mapa"
for solver in "${solvers[@]}"; do
    printf " | %-${solver_column_width}s" "$solver"
done
printf '\n'

separator_width=$((map_column_width + (${#solvers[@]} * (solver_column_width + 3))))
printf '%*s\n' "$separator_width" '' | tr ' ' '-'

for map_name in "${maps[@]}"; do
    printf "%-${map_column_width}s" "$map_name"
    for solver in "${solvers[@]}"; do
        printf " | %${solver_column_width}d" "${maximum_agents["$map_name|$solver"]}"
    done
    printf '\n'
done

total_elapsed_seconds=$((SECONDS - start_seconds))
printf '\nObservação: um aviso indica quando o valor encontrado está limitado pela quantidade de agentes do cenário.\n'
printf '\nTempo total: %d segundos.\n' "$total_elapsed_seconds"
