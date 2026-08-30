#!/usr/bin/env bash
set -eo pipefail

# End-to-end physical acceptance with independent arm and base-motion budgets.
# This prevents a navigation regression from hiding behind faster manipulation,
# or the reverse.

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
max_sim_seconds="${MAX_MANIPULATION_SIM_SECONDS:-130}"
max_base_sim_seconds="${MAX_BASE_SIM_SECONDS:-70}"
timing_log="/tmp/physical_order_truth.log"

if ! [[ "${max_sim_seconds}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "MAX_MANIPULATION_SIM_SECONDS must be a positive number"
  exit 2
fi
if ! [[ "${max_base_sim_seconds}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "MAX_BASE_SIM_SECONDS must be a positive number"
  exit 2
fi

export HEADLESS="${HEADLESS:-false}"
export RAW_BIN_SEED="${RAW_BIN_SEED:-17}"
export ORDER_QUANTITY=1
export ALLOWED_MACHINES="${ALLOWED_MACHINES:-machine_2}"

echo "[performance] Running one randomized, contact-verified factory cycle"
echo "[performance] Gazebo GUI: $([[ "${HEADLESS}" == "false" ]] && echo enabled || echo disabled)"
echo "[performance] Manipulation simulation-time budget: ${max_sim_seconds} s"
echo "[performance] Base-motion simulation-time budget: ${max_base_sim_seconds} s"

"${project_root}/scripts/test_sparse_bin_factory_cycle_truth.sh"

[[ -s "${timing_log}" ]] || {
  echo "Physical-order timing log is missing: ${timing_log}"
  exit 1
}

awk \
  -v manipulation_limit="${max_sim_seconds}" \
  -v base_limit="${max_base_sim_seconds}" '
  function is_manipulation_phase(phase) {
    return phase == "pick_raw_bin" ||
      phase ~ /^place_machine_[123]$/ ||
      phase ~ /^pick_machine_[123]$/ ||
      phase == "place_finished_bin"
  }

  function is_base_phase(phase) {
    return phase ~ /^dock_/ || phase ~ /^undock_/
  }

  /Physical step [0-9]+\/[0-9]+: .* completed in/ {
    line = $0
    phase = line
    sub(/^.*Physical step [0-9]+\/[0-9]+: /, "", phase)
    sub(/ completed in.*$/, "", phase)
    manipulation = is_manipulation_phase(phase)
    base = is_base_phase(phase)
    if (!manipulation && !base) {
      next
    }

    wall = line
    sub(/^.* completed in /, "", wall)
    sub(/ s wall,.*$/, "", wall)
    sim = line
    sub(/^.* s wall, /, "", sim)
    sub(/ s simulation.*$/, "", sim)

    if (wall !~ /^[0-9]+([.][0-9]+)?$/ ||
        sim !~ /^[0-9]+([.][0-9]+)?$/) {
      printf "Could not parse timing for %s\n", phase > "/dev/stderr"
      parse_failed = 1
      next
    }
    if (manipulation) {
      printf "[performance] arm  %-20s wall=%6.1f s  simulation=%6.1f s\n",
        phase, wall, sim
      manipulation_wall += wall
      manipulation_sim += sim
      manipulation_count += 1
    }
    if (base) {
      printf "[performance] base %-20s wall=%6.1f s  simulation=%6.1f s\n",
        phase, wall, sim
      base_wall += wall
      base_sim += sim
      base_count += 1
    }
  }

  END {
    if (parse_failed || manipulation_count != 4 || manipulation_sim <= 0.0) {
      printf "Expected four valid manipulation timings, found %d\n",
        manipulation_count > "/dev/stderr"
      exit 1
    }
    if (base_count != 8 || base_sim <= 0.0) {
      printf "Expected eight valid base-motion timings, found %d\n",
        base_count > "/dev/stderr"
      exit 1
    }
    printf "[performance] %-24s wall=%6.1f s  simulation=%6.1f s\n",
      "four arm phases", manipulation_wall, manipulation_sim
    printf "[performance] %-24s wall=%6.1f s  simulation=%6.1f s\n",
      "eight base phases", base_wall, base_sim
    if (manipulation_sim > manipulation_limit) {
      printf "Manipulation exceeded %.1f s simulation-time budget\n",
        manipulation_limit > "/dev/stderr"
      exit 1
    }
    printf "manipulation_performance_success=true simulation_seconds=%.1f budget_seconds=%.1f\n",
      manipulation_sim, manipulation_limit
    if (base_sim > base_limit) {
      printf "Base motion exceeded %.1f s simulation-time budget\n",
        base_limit > "/dev/stderr"
      exit 1
    }
    printf "base_performance_success=true simulation_seconds=%.1f budget_seconds=%.1f\n",
      base_sim, base_limit
  }
' "${timing_log}"
