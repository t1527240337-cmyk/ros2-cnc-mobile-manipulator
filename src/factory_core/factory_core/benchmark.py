from __future__ import annotations

import argparse
import csv
import random
from pathlib import Path

from .domain import FactoryState, ProductionOrder
from .scheduler import DecisionKind, SimulationEngine


def run_trial(seed: int, quantity: int = 3) -> dict[str, object]:
    rng = random.Random(seed)
    state = FactoryState.default(cycle_seconds=rng.uniform(8.0, 16.0))
    state.battery = rng.uniform(0.18, 1.0)
    faulted_machine = ""
    if rng.random() < 0.25:
        faulted_machine = rng.choice(list(state.machines))
        state.machines[faulted_machine].inject_fault(50)
    order = ProductionOrder(f"benchmark-{seed}", quantity, list(state.machines))
    result = SimulationEngine(state, order).run()
    return {
        "seed": seed,
        "success": result.kind == DecisionKind.COMPLETE,
        "completed": order.completed,
        "quantity": quantity,
        "simulated_seconds": round(state.simulated_time, 3),
        "final_battery": round(state.battery, 4),
        "faulted_machine": faulted_machine,
        "result": result.kind.value,
        "detail": result.detail,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Run deterministic seeded semantic benchmarks")
    parser.add_argument("--trials", type=int, default=30)
    parser.add_argument("--quantity", type=int, default=3)
    parser.add_argument("--output", type=Path, default=Path("benchmark_results.csv"))
    args = parser.parse_args()
    rows = [run_trial(seed, args.quantity) for seed in range(args.trials)]
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    successes = sum(bool(row["success"]) for row in rows)
    print(f"success={successes}/{len(rows)} ({successes / len(rows):.1%}) output={args.output}")


if __name__ == "__main__":
    main()
