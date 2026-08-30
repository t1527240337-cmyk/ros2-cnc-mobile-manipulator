from __future__ import annotations

import argparse
import uuid

from .domain import FactoryState, ProductionOrder
from .scheduler import SimulationEngine


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the deterministic three-machine demo")
    parser.add_argument("--quantity", type=int, default=3)
    parser.add_argument("--battery", type=float, default=1.0)
    args = parser.parse_args()

    state = FactoryState.default()
    state.battery = args.battery
    order = ProductionOrder(
        order_id=f"demo-{uuid.uuid4().hex[:8]}",
        quantity=args.quantity,
        allowed_machine_ids=list(state.machines),
    )
    engine = SimulationEngine(state, order)
    result = engine.run()
    for event in state.events:
        print(event)
    print(f"result={result.kind.value} completed={order.completed}/{order.quantity} "
          f"battery={state.battery:.1%} simulated_time={state.simulated_time:.1f}s")


if __name__ == "__main__":
    main()
