"""Resumable fixed-seed acceptance for the deterministic physical stack."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time
from typing import Iterable


DEFAULT_SEED_SPEC = "101-130"
DEFAULT_SUCCESS_RATE = 0.80
MINIMUM_FORMAL_SEEDS = 30
CAMPAIGN_SCHEMA_VERSION = 40
CAMPAIGN_LAYOUT = "v45"
TERMINAL_STATUSES = frozenset({"passed", "failed"})
PHYSICAL_ARTIFACTS = (
    Path("/tmp/physical_order_truth.log"),
    Path("/tmp/physical_order_result.log"),
    Path("/tmp/physical_order_summary.log"),
    Path("/tmp/physical_order_factory_state.log"),
    Path("/tmp/physical_order_machine_states.log"),
)


@dataclass(frozen=True)
class SeedScenario:
    """One reproducible, non-Agent production acceptance case."""

    seed: int
    profile: str
    quantity: int
    raw_part_count: int
    allowed_machines: tuple[str, ...]
    initial_battery: float
    expect_recharge: bool
    robot_x: float
    robot_y: float
    robot_yaw: float
    fault_machine_before_order: str | None = None
    expected_execution_machine: str | None = None

    def environment(self, *, headless: bool) -> dict[str, str]:
        values = {
            "HEADLESS": str(headless).lower(),
            "SPARSE_BIN": "true",
            "FINISHED_SLOT_PERCEPTION": "true",
            "RAW_BIN_SEED": str(self.seed),
            "RAW_PART_COUNT": str(self.raw_part_count),
            "ORDER_ID": f"physical_seed_{self.seed}",
            "ORDER_QUANTITY": str(self.quantity),
            "ALLOWED_MACHINES": ",".join(self.allowed_machines),
            "INITIAL_BATTERY_PERCENTAGE": f"{self.initial_battery:.2f}",
            "AUTO_RECHARGE": "true",
            "EXPECT_RECHARGE": str(self.expect_recharge).lower(),
            "ROBOT_X": f"{self.robot_x:.3f}",
            "ROBOT_Y": f"{self.robot_y:.3f}",
            "ROBOT_YAW": f"{self.robot_yaw:.3f}",
        }
        if self.fault_machine_before_order is not None:
            values["FAULT_MACHINE_BEFORE_ORDER"] = (
                self.fault_machine_before_order
            )
        if self.expected_execution_machine is not None:
            values["EXPECTED_EXECUTION_MACHINE"] = (
                self.expected_execution_machine
            )
        return values


def parse_seed_spec(specification: str) -> tuple[int, ...]:
    """Parse ``101-105,110`` into a sorted, duplicate-free seed tuple."""
    seeds: set[int] = set()
    for raw_item in specification.split(","):
        item = raw_item.strip()
        if not item:
            raise ValueError("seed list contains an empty item")
        if "-" in item:
            start_text, end_text = item.split("-", maxsplit=1)
            start = int(start_text)
            end = int(end_text)
            if end < start:
                raise ValueError(f"descending seed range is invalid: {item}")
            seeds.update(range(start, end + 1))
        else:
            seeds.add(int(item))
    if not seeds or min(seeds) < 0:
        raise ValueError("at least one non-negative seed is required")
    return tuple(sorted(seeds))


def _initial_pose_for_seed(seed: int) -> tuple[float, float, float]:
    """Generate bounded, deterministic map-start perturbations from one seed."""
    x = 0.012 * (((seed * 17) % 11) - 5)
    y = -1.2 + 0.012 * (((seed * 23) % 11) - 5)
    yaw = 0.012 * (((seed * 29) % 9) - 4)
    return round(x, 3), round(y, 3), round(yaw, 3)


def scenario_for_seed(seed: int, profile: str) -> SeedScenario:
    """Map a seed to a documented physical scenario without hidden randomness."""
    quantity = 2 if seed % 6 == 0 else 1
    raw_part_count = max(quantity, 3 + seed % 4)
    if seed < 0:
        raise ValueError("seed must be non-negative")
    robot_x, robot_y, robot_yaw = _initial_pose_for_seed(seed)
    if profile == "three-machine":
        return SeedScenario(
            seed=seed,
            profile=profile,
            quantity=3,
            raw_part_count=6,
            allowed_machines=("machine_1", "machine_2", "machine_3"),
            initial_battery=0.42,
            expect_recharge=False,
            robot_x=robot_x,
            robot_y=robot_y,
            robot_yaw=robot_yaw,
        )
    if profile != "production":
        raise ValueError(f"unknown campaign profile: {profile}")

    primary_number = ((seed - 1) % 3) + 1
    primary = f"machine_{primary_number}"
    low_battery = seed % 5 == 0
    if seed % 7 != 0:
        return SeedScenario(
            seed=seed,
            profile=profile,
            quantity=quantity,
            raw_part_count=raw_part_count,
            allowed_machines=(primary,),
            initial_battery=0.20 if low_battery else 0.42,
            expect_recharge=low_battery,
            robot_x=robot_x,
            robot_y=robot_y,
            robot_yaw=robot_yaw,
        )

    reroute = f"machine_{(primary_number % 3) + 1}"
    return SeedScenario(
        seed=seed,
        profile=profile,
        quantity=quantity,
        raw_part_count=raw_part_count,
        allowed_machines=(primary, reroute),
        initial_battery=0.20 if low_battery else 0.42,
        expect_recharge=low_battery,
        robot_x=robot_x,
        robot_y=robot_y,
        robot_yaw=robot_yaw,
        fault_machine_before_order=primary,
        expected_execution_machine=reroute,
    )


def parse_physical_summary(text: str) -> dict[str, str]:
    """Read the stable ``key=value`` contract from a physical test."""
    fields: dict[str, str] = {}
    for token in shlex.split(text.strip()):
        if "=" not in token:
            continue
        key, value = token.split("=", maxsplit=1)
        fields[key] = value
    if fields.get("physics") != "contact_verified":
        raise ValueError("summary does not prove contact-verified physics")
    if fields.get("physical_order_success") != "true":
        raise ValueError("summary does not report a successful order")
    return fields


def parse_raw_targets(text: str) -> tuple[tuple[float, float, float], ...]:
    """Parse a semicolon-delimited sensor target path from acceptance output."""
    if not text:
        raise ValueError("raw target path is empty")
    targets: list[tuple[float, float, float]] = []
    for item in text.split(";"):
        components = item.split(",")
        if len(components) != 3:
            raise ValueError(f"invalid raw target coordinate: {item!r}")
        target = tuple(float(component) for component in components)
        if not all(math.isfinite(component) for component in target):
            raise ValueError("raw target coordinates must be finite")
        x, y, z = target
        if not (
            0.55 <= x <= 1.00
            and -0.45 <= y <= 0.45
            and 0.18 <= z <= 0.38
        ):
            raise ValueError(
                f"raw target is outside the measured work volume: {target}"
            )
        targets.append((x, y, z))
    return tuple(targets)


def validate_summary_for_scenario(
    fields: dict[str, str], scenario: SeedScenario
) -> None:
    """Bind a successful summary to the exact deterministic seed scenario."""
    expected = {
        "order": f"physical_seed_{scenario.seed}",
        "completed": str(scenario.quantity),
        "machines": ",".join(scenario.allowed_machines),
        "initial_raw_inventory": str(scenario.raw_part_count),
        "raw_inventory": str(scenario.raw_part_count - scenario.quantity),
        "finished_inventory": str(scenario.quantity),
        "orchestration": "action",
        "physics": "contact_verified",
        "fault_isolated": scenario.fault_machine_before_order or "none",
        "raw_source": "rgbd_sparse_bin",
        "raw_seed": str(scenario.seed),
        "raw_layout": "unordered_workspace",
        "raw_target_count": str(scenario.quantity),
        "finished_source": "rgbd_slots",
        "recharge_verified": str(scenario.expect_recharge).lower(),
        "robot_spawn": (
            f"{scenario.robot_x:.3f},{scenario.robot_y:.3f},"
            f"{scenario.robot_yaw:.3f}"
        ),
    }
    if scenario.expected_execution_machine is not None:
        expected["execution_machine"] = scenario.expected_execution_machine
    elif len(scenario.allowed_machines) == 1:
        expected["execution_machine"] = scenario.allowed_machines[0]

    mismatches = [
        f"{key}: expected {value!r}, got {fields.get(key)!r}"
        for key, value in expected.items()
        if fields.get(key) != value
    ]
    try:
        targets = parse_raw_targets(fields.get("raw_targets", ""))
    except ValueError as error:
        mismatches.append(f"raw_targets: {error}")
    else:
        if len(targets) != scenario.quantity:
            mismatches.append(
                "raw_targets: expected "
                f"{scenario.quantity} targets, got {len(targets)}"
            )
    if mismatches:

        raise ValueError(
            "physical summary does not match scenario; "
            + "; ".join(mismatches)
        )

def _scenario_document(scenario: SeedScenario) -> dict[str, object]:
    """Return the exact JSON representation stored in evidence records."""
    return json.loads(json.dumps(asdict(scenario)))


def _scenario_digest(scenario: SeedScenario) -> str:
    payload = json.dumps(
        _scenario_document(scenario), sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _file_digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _evidence_manifest(seed_directory: Path) -> dict[str, dict[str, object]]:
    manifest: dict[str, dict[str, object]] = {}
    candidates = (seed_directory / "console.log",) + tuple(
        seed_directory / source.name for source in PHYSICAL_ARTIFACTS
    )
    for path in candidates:
        if path.is_file():
            manifest[path.name] = {
                "bytes": path.stat().st_size,
                "sha256": _file_digest(path),
            }
    return manifest


def _validate_result_record(
    path: Path, record: dict[str, object], scenario: SeedScenario
) -> None:
    """Reject stale, edited, or scenario-incompatible resumable evidence."""
    if record.get("schema_version") != CAMPAIGN_SCHEMA_VERSION:
        raise ValueError(
            f"{path} uses an incompatible evidence schema; "
            f"use the {CAMPAIGN_LAYOUT} result directory"
        )
    if (
        record.get("seed") != scenario.seed
        or record.get("profile") != scenario.profile
    ):
        raise ValueError(f"{path} top-level seed/profile is invalid")
    expected_scenario = _scenario_document(scenario)
    if record.get("scenario") != expected_scenario:
        raise ValueError(f"{path} scenario does not match seed {scenario.seed}")
    if record.get("scenario_digest") != _scenario_digest(scenario):
        raise ValueError(f"{path} scenario digest is invalid")
    if record.get("status") not in TERMINAL_STATUSES:
        raise ValueError(f"{path} has a non-terminal status")

    manifest = record.get("evidence_manifest")
    if not isinstance(manifest, dict) or "console.log" not in manifest:
        raise ValueError(f"{path} has no complete evidence manifest")
    if record.get("status") == "passed":
        required = {
            "physical_order_summary.log",
            "physical_order_truth.log",
            "physical_order_result.log",
            "physical_order_factory_state.log",
            "physical_order_machine_states.log",
        }
        missing = required.difference(manifest)
        if missing:
            raise ValueError(f"{path} passed without evidence: {sorted(missing)}")

    for name, metadata in manifest.items():
        if not isinstance(name, str) or Path(name).name != name:
            raise ValueError(f"{path} contains an invalid evidence filename")
        if not isinstance(metadata, dict):
            raise ValueError(f"{path} contains invalid evidence metadata for {name}")
        evidence = path.parent / name
        if not evidence.is_file():
            raise ValueError(f"{path} evidence is missing: {name}")
        if metadata.get("bytes") != evidence.stat().st_size:
            raise ValueError(f"{path} evidence size changed: {name}")
        if metadata.get("sha256") != _file_digest(evidence):
            raise ValueError(f"{path} evidence digest changed: {name}")

    if record.get("status") == "passed":
        copied_summary = parse_physical_summary(
            (path.parent / "physical_order_summary.log").read_text(encoding="utf-8")
        )
        validate_summary_for_scenario(copied_summary, scenario)


def campaign_summary(
    seeds: Iterable[int],
    results: dict[int, dict[str, object]],
    minimum_success_rate: float,
) -> dict[str, object]:
    """Aggregate terminal seed records without overstating partial runs."""
    configured = tuple(seeds)
    terminal = {
        seed: result
        for seed, result in results.items()
        if seed in configured and result.get("status") in TERMINAL_STATUSES
    }
    passed = sum(
        result.get("status") == "passed" for result in terminal.values()
    )
    failed = sum(
        result.get("status") == "failed" for result in terminal.values()
    )
    total = len(configured)
    completed = len(terminal)
    required_successes = math.ceil(total * minimum_success_rate)
    complete = completed == total
    formal_eligible = total >= MINIMUM_FORMAL_SEEDS
    formal_complete = complete and formal_eligible

    raw_targets: list[tuple[float, float, float]] = []
    for result in terminal.values():
        if result.get("status") != "passed":
            continue
        fields = result.get("physical_summary")
        if not isinstance(fields, dict):
            continue
        try:
            raw_targets.extend(
                parse_raw_targets(str(fields.get("raw_targets", "")))
            )
        except ValueError:
            continue
    target_cells = {
        (math.floor(x / 0.05), math.floor(y / 0.05))
        for x, y, _z in raw_targets
    }
    x_span = (
        max(target[0] for target in raw_targets)
        - min(target[0] for target in raw_targets)
        if raw_targets
        else 0.0
    )
    y_span = (
        max(target[1] for target in raw_targets)
        - min(target[1] for target in raw_targets)
        if raw_targets
        else 0.0
    )
    threshold_met = (
        passed >= required_successes if formal_complete else None
    )
    coverage_met = (
        len(target_cells) >= 12 and x_span >= 0.10 and y_span >= 0.30
        if formal_complete
        else None
    )
    return {
        "configured_seeds": list(configured),
        "total": total,
        "completed": completed,
        "passed": passed,
        "failed": failed,
        "pending": total - completed,
        "required_successes": required_successes,
        "formal_eligible": formal_eligible,
        "all_configured_passed": complete and failed == 0,
        "completed_success_rate": (
            passed / completed if completed else None
        ),
        "formal_success_rate": passed / total if formal_complete else None,
        "threshold_met": threshold_met,
        "raw_target_samples": len(raw_targets),
        "raw_target_cells_50mm": len(target_cells),
        "raw_target_x_span": round(x_span, 4),
        "raw_target_y_span": round(y_span, 4),
        "unordered_coverage_met": coverage_met,
        "qualification_met": (
            threshold_met and coverage_met if formal_complete else None
        ),
    }


def _atomic_json(path: Path, document: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _load_result(path: Path) -> dict[str, object] | None:
    if not path.exists():
        return None
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"result is not a JSON object: {path}")
    return document


def _acquire_campaign_lock(result_root: Path):
    """Prevent concurrent runners from overwriting one seed's evidence."""
    lock_path = result_root / ".campaign.lock"
    lock_handle = lock_path.open("w", encoding="utf-8")
    try:
        fcntl.flock(
            lock_handle.fileno(),
            fcntl.LOCK_EX | fcntl.LOCK_NB,
        )
    except BlockingIOError as error:
        lock_handle.close()
        raise RuntimeError(
            f"another physical seed campaign owns {result_root}"
        ) from error
    lock_handle.write(f"pid={os.getpid()}\n")
    lock_handle.flush()
    return lock_handle


def _clear_physical_artifacts() -> None:
    """Prevent a failed seed from inheriting another seed's /tmp evidence."""
    for path in PHYSICAL_ARTIFACTS:
        path.unlink(missing_ok=True)


def _copy_physical_artifacts(seed_directory: Path) -> None:
    for source in PHYSICAL_ARTIFACTS:
        if source.is_file():
            shutil.copy2(source, seed_directory / source.name)


def _run_and_tee(
    command: list[str],
    *,
    cwd: Path,
    environment: dict[str, str],
    output_path: Path,
) -> int:
    with output_path.open("w", encoding="utf-8") as output:
        process = subprocess.Popen(
            command,
            cwd=cwd,
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert process.stdout is not None
        try:
            for line in process.stdout:
                print(line, end="", flush=True)
                output.write(line)
                output.flush()
        except KeyboardInterrupt:
            process.terminate()
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                process.kill()
            raise
        return process.wait()


def run_seed(
    scenario: SeedScenario,
    *,
    project_root: Path,
    result_root: Path,
    headless: bool,
) -> dict[str, object]:
    """Run one seed, preserve all evidence, and return a terminal record."""
    seed_directory = result_root / f"seed_{scenario.seed}"
    seed_directory.mkdir(parents=True, exist_ok=True)
    console_log = seed_directory / "console.log"
    test_script = project_root / "scripts" / "test_sparse_bin_factory_cycle_truth.sh"
    if not test_script.is_file():
        raise FileNotFoundError(f"physical test script is missing: {test_script}")

    environment = os.environ.copy()
    environment.update(scenario.environment(headless=headless))
    _clear_physical_artifacts()
    started_at = time.monotonic()
    return_code = _run_and_tee(
        [str(test_script)],
        cwd=project_root,
        environment=environment,
        output_path=console_log,
    )
    wall_seconds = round(time.monotonic() - started_at, 3)
    _copy_physical_artifacts(seed_directory)

    if return_code == 75:
        raise RuntimeError(
            "another physical acceptance owns the ROS domain; no seed result "
            "was recorded"
        )

    summary_path = Path("/tmp/physical_order_summary.log")
    fields: dict[str, str] = {}
    parse_error = ""
    if return_code == 0:
        try:
            fields = parse_physical_summary(
                summary_path.read_text(encoding="utf-8")
            )
            validate_summary_for_scenario(fields, scenario)
        except (OSError, ValueError) as error:
            parse_error = str(error)

    passed = return_code == 0 and not parse_error
    evidence_manifest = _evidence_manifest(seed_directory)
    record: dict[str, object] = {
        "schema_version": CAMPAIGN_SCHEMA_VERSION,
        "recorded_at": datetime.now(timezone.utc).isoformat(),
        "status": "passed" if passed else "failed",
        "seed": scenario.seed,
        "profile": scenario.profile,
        "scenario": _scenario_document(scenario),
        "scenario_digest": _scenario_digest(scenario),
        "evidence_manifest": evidence_manifest,
        "exit_code": return_code,
        "runner_wall_seconds": wall_seconds,
        "physical_summary": fields,
        "parse_error": parse_error,
    }
    _atomic_json(seed_directory / "result.json", record)
    return record


def _read_campaign_results(
    seeds: Iterable[int], result_root: Path, profile: str
) -> dict[int, dict[str, object]]:
    results: dict[int, dict[str, object]] = {}
    for seed in seeds:
        path = result_root / f"seed_{seed}" / "result.json"
        record = _load_result(path)
        if record is None:
            continue
        if record.get("profile") != profile:
            raise ValueError(
                f"{path} belongs to profile {record.get('profile')!r}; "
                f"use a separate result directory for {profile!r}"
            )
        _validate_result_record(path, record, scenario_for_seed(seed, profile))
        results[seed] = record
    return results


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run resumable, contact-verified Gazebo production seeds. "
            "Agent and MCP are intentionally outside this campaign."
        )
    )
    parser.add_argument("--seeds", default=DEFAULT_SEED_SPEC)
    parser.add_argument(
        "--profile",
        choices=("production", "three-machine"),
        default="production",
    )
    parser.add_argument("--project-root", type=Path, default=Path.cwd())
    parser.add_argument("--result-dir", type=Path)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument(
        "--verify-only",
        action="store_true",
        help="validate saved evidence and summarize it without starting Gazebo",
    )
    parser.add_argument("--rerun-failed", action="store_true")
    parser.add_argument("--stop-on-failure", action="store_true")
    parser.add_argument(
        "--minimum-success-rate",
        type=float,
        default=DEFAULT_SUCCESS_RATE,
    )
    return parser


def main(arguments: list[str] | None = None) -> int:
    options = _build_parser().parse_args(arguments)
    if not 0.0 < options.minimum_success_rate <= 1.0:
        raise ValueError("minimum success rate must be in (0, 1]")

    seeds = parse_seed_spec(options.seeds)
    project_root = options.project_root.expanduser().resolve()
    result_root = (
        options.result_dir.expanduser().resolve()
        if options.result_dir is not None
        else (
            project_root
            / "artifacts"
            / "physical_seed_campaign"
            / CAMPAIGN_LAYOUT
            / options.profile
        )
    )
    result_root.mkdir(parents=True, exist_ok=True)
    campaign_lock = _acquire_campaign_lock(result_root)
    results = _read_campaign_results(seeds, result_root, options.profile)

    print(
        "physical_seed_campaign "
        f"profile={options.profile} seeds={options.seeds} "
        f"result_dir={result_root}",
        flush=True,
    )
    for seed in seeds:
        previous = results.get(seed)
        previous_status = previous.get("status") if previous else None
        should_resume = previous_status == "passed" or (
            previous_status == "failed" and not options.rerun_failed
        )
        if should_resume:
            print(
                f"seed={seed} status={previous_status} action=resume",
                flush=True,
            )
            continue
        if options.verify_only:
            print(f"seed={seed} status=pending action=verify", flush=True)
            continue

        scenario = scenario_for_seed(seed, options.profile)
        print(
            f"seed={seed} action=run scenario="
            f"{json.dumps(asdict(scenario), ensure_ascii=False)}",
            flush=True,
        )
        record = run_seed(
            scenario,
            project_root=project_root,
            result_root=result_root,
            headless=not options.gui,
        )
        results[seed] = record
        print(
            f"seed={seed} status={record['status']} "
            f"wall_seconds={record['runner_wall_seconds']}",
            flush=True,
        )
        if record["status"] == "failed" and options.stop_on_failure:
            break

    results = _read_campaign_results(seeds, result_root, options.profile)
    summary = campaign_summary(seeds, results, options.minimum_success_rate)
    summary.update(
        {
            "schema_version": CAMPAIGN_SCHEMA_VERSION,
            "evidence_layout": CAMPAIGN_LAYOUT,
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "profile": options.profile,
            "minimum_success_rate": options.minimum_success_rate,
            "result_directory": str(result_root),
        }
    )
    _atomic_json(result_root / "summary.json", summary)
    print(
        "campaign_summary "
        f"completed={summary['completed']}/{summary['total']} "
        f"passed={summary['passed']} failed={summary['failed']} "
        f"pending={summary['pending']} "
        f"threshold_met={summary['threshold_met']}",
        flush=True,
    )

    campaign_lock.close()
    if summary["pending"]:
        return 2
    if summary["formal_eligible"]:
        return 0 if summary["qualification_met"] else 1
    return 0 if summary["all_configured_passed"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("physical seed campaign interrupted", file=sys.stderr)
        sys.exit(130)
    except (OSError, RuntimeError, ValueError) as error:
        print(f"physical seed campaign failed: {error}", file=sys.stderr)
        sys.exit(2)
