#!/usr/bin/env python3
"""Summarize BehaviorTree leaf reliability from physical campaign evidence."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
from pathlib import Path
import re
import statistics


STEP_PATTERN = re.compile(
    r"BT physical step (?P<name>[A-Za-z0-9_]+) "
    r"(?P<status>completed in|failed after) "
    r"(?P<seconds>[0-9.]+) s"
)


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(round((len(ordered) - 1) * fraction)))
    return ordered[index]


def analyze(campaign_dir: Path) -> dict:
    samples = defaultdict(lambda: {"success": [], "failure": []})
    seed_runs = []
    for log_file in sorted(campaign_dir.glob("seed_*/console.log")):
        for match in STEP_PATTERN.finditer(
            log_file.read_text(encoding="utf-8", errors="replace")
        ):
            status = (
                "success"
                if match.group("status") == "completed in"
                else "failure"
            )
            samples[match.group("name")][status].append(
                float(match.group("seconds"))
            )

    for result_file in sorted(campaign_dir.glob("seed_*/result.json")):
        result = json.loads(result_file.read_text(encoding="utf-8"))
        wall_seconds = result.get("runner_wall_seconds")
        if not isinstance(wall_seconds, (int, float)):
            continue
        seed_runs.append({
            "seed": result.get("seed"),
            "status": result.get("status"),
            "quantity": result.get("scenario", {}).get("quantity"),
            "wall_seconds": float(wall_seconds),
        })
    steps = {}
    for name, values in sorted(samples.items()):
        successes = len(values["success"])
        failures = len(values["failure"])
        attempts = successes + failures
        durations = values["success"] + values["failure"]
        steps[name] = {
            "attempts": attempts,
            "successes": successes,
            "failures": failures,
            "success_rate": successes / attempts if attempts else None,
            "median_seconds": statistics.median(durations) if durations else None,
            "p95_seconds": percentile(durations, 0.95),
        }

    summary_file = campaign_dir / "summary.json"
    campaign_summary = (
        json.loads(summary_file.read_text(encoding="utf-8"))
        if summary_file.exists()
        else None
    )
    all_wall = [item["wall_seconds"] for item in seed_runs]
    passed_wall = [
        item["wall_seconds"]
        for item in seed_runs
        if item["status"] == "passed"
    ]
    return {
        "campaign_dir": str(campaign_dir.resolve()),
        "campaign_summary": campaign_summary,
        "seed_runtime": {
            "attempts": len(seed_runs),
            "passed_attempts": len(passed_wall),
            "median_wall_seconds": (
                statistics.median(all_wall) if all_wall else None
            ),
            "p95_wall_seconds": percentile(all_wall, 0.95),
            "passed_median_wall_seconds": (
                statistics.median(passed_wall) if passed_wall else None
            ),
        },
        "bt_leaf_steps": steps,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("campaign_dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    result = analyze(args.campaign_dir)
    rendered = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")


if __name__ == "__main__":
    main()
