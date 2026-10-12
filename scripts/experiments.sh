#!/usr/bin/env bash

set -euo pipefail

usage() {
    cat <<'EOF'
Uso:
  scripts/experiments.sh [normal|profile] [--dry-run]

Executa a matriz de mapas, cenários, solvers, agentes e estratégias.
O modo padrão é normal. Compile o executável correspondente antes de executar.

  --dry-run  Mostra todos os comandos, sem executar os solvers.
  -h, --help Mostra esta ajuda.

Os experimentos são sequenciais e salvam os CSVs em results/ pelo mapf_app.
Uma falha é informada no terminal e não interrompe os próximos experimentos.
Códigos de saída: 0 = todos concluídos com sucesso; 1 = algum retorno 1 da CLI
(sem solução ou falha na gravação); 2 = erro de configuração ou execução.
EOF
}

mode=normal
mode_specified=false
dry_run=false
for argument in "$@"; do
    case "$argument" in
        normal|profile)
            if $mode_specified; then
                echo "Erro: informe o modo de compilação apenas uma vez." >&2
                exit 2
            fi
            mode=$argument
            mode_specified=true
            ;;
        --dry-run) dry_run=true ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Erro: argumento desconhecido: $argument" >&2; usage >&2; exit 2 ;;
    esac
done

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd -- "$script_dir/.." && pwd)
cd "$repository_root"

# Cada linha associa um mapa às suas três quantidades de agentes.
# O cenário correspondente é sempre <mapa>/random/<mapa>-random-1.scen.
map_configs=(
    "den520d 500 700 900"
    "empty-8-8 8 16 32"
    "empty-32-32 300 400 500"
    "random-32-32-20 50 100 150"
    "warehouse-10-20-10-2-1 150 250 350"
    "ost003d 100 300 500"
)
solvers=(
    PriorityPlanningSolver
    LocalPathRepairIterativeSolver
    FullPathRepairIterativeSolver
)
strategies=(RESOLVE_BY_AGENT RESOLVE_BY_TIME)

# Por mapa/agentes: 1 Priority + 2 Iterative + 1 Full.
total_experiments=$(( ${#map_configs[@]} * 3 * (1 + ${#strategies[@]} + 1) ))

if ! $dry_run && [[ ! -x build/$mode/mapf_app ]]; then
    echo "Erro: build/$mode/mapf_app não existe ou não é executável." >&2
    echo "Compile com: cmake --preset $mode && cmake --build --preset $mode --parallel" >&2
    exit 2
fi

# Confira todas as entradas antes de começar a bateria.
for config in "${map_configs[@]}"; do
    read -r map_name first_count second_count max_count <<< "$config"
    map="benchmarks/maps/$map_name.map"
    scenario="benchmarks/scenarios/$map_name/random/$map_name-random-1.scen"
    for input_file in "$map" "$scenario"; do
        if [[ ! -r $input_file ]]; then
            echo "Erro: arquivo de entrada indisponível: $input_file" >&2
            exit 2
        fi
    done
    available_agents=$(awk 'NR > 1 && NF { count++ } END { print count + 0 }' "$scenario")
    if (( max_count > available_agents )); then
        echo "Aviso: $scenario contém $available_agents agentes; pedidos acima desse limite serão rejeitados pela CLI. A matriz foi mantida." >&2
    fi
done

trap 'printf "\nBateria interrompida.\n" >&2; exit 130' INT
trap 'printf "\nBateria interrompida.\n" >&2; exit 143' TERM

current=0
succeeded=0
unsuccessful=0
errors=0
printf 'Total: %d experimentos; modo: %s; dry-run: %s\n' "$total_experiments" "$mode" "$dry_run"

for config in "${map_configs[@]}"; do
    read -r map_name first_count second_count third_count <<< "$config"
    map="benchmarks/maps/$map_name.map"
    scenario="benchmarks/scenarios/$map_name/random/$map_name-random-1.scen"

    for solver in "${solvers[@]}"; do
        # A opção vazia representa uma única execução sem essa flag.
        solver_strategies=("")
        case "$solver" in
            LocalPathRepairIterativeSolver)
                solver_strategies=("${strategies[@]}")
                ;;
        esac

        for agents in "$first_count" "$second_count" "$third_count"; do
            for strategy in "${solver_strategies[@]}"; do
                command=(scripts/run_experiment.sh "$mode" --
                    -map "$map" -scen "$scenario" -solver "$solver" -agents "$agents")
                if [[ -n $strategy ]]; then
                    command+=(-localRepairStrategy "$strategy")
                fi

                current=$((current + 1))
                printf '\n[%d/%d] ' "$current" "$total_experiments"
                printf '%q ' "${command[@]}"
                printf '\n'
                if $dry_run; then
                    continue
                fi

                if "${command[@]}"; then
                    exit_code=0
                else
                    exit_code=$?
                fi
                case "$exit_code" in
                    0) succeeded=$((succeeded + 1)); echo "Resultado: sucesso." ;;
                    1)
                        unsuccessful=$((unsuccessful + 1))
                        echo "Resultado: sem solução ou falha na gravação dos artefatos (código 1)."
                        ;;
                    130|143) exit "$exit_code" ;;
                    *) errors=$((errors + 1)); echo "Resultado: erro (código $exit_code)." ;;
                esac
            done
        done
    done
done

if $dry_run; then
    printf '\n%d comandos listados; nenhum experimento executado.\n' "$current"
    exit 0
fi

printf '\nBateria concluída: %d execuções; %d sucessos; %d retornos 1; %d erros.\n' \
    "$current" "$succeeded" "$unsuccessful" "$errors"
if (( errors > 0 )); then
    exit 2
fi
if (( unsuccessful > 0 )); then
    exit 1
fi
exit 0
