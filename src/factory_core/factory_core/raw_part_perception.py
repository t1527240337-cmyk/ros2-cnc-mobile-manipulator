"""Readable client-side selection for anonymous raw-bin observations."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from statistics import median
import threading
import time

from geometry_msgs.msg import PoseArray
from rclpy.node import Node


@dataclass(frozen=True)
class PerceivedPart:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class PerceivedSelection:
    """One stable target plus the fresh candidate set used to select it."""

    target: PerceivedPart
    candidates: tuple[PerceivedPart, ...]
    stable_candidates: tuple[PerceivedPart, ...]


class StableCandidateCollector:
    """Wait briefly for a complete multi-view candidate set.

    A target can become temporally stable on the primary camera immediately
    before an auxiliary-camera callback adds another visible workpiece. The
    collector waits for a bounded number of *new* fused frames and retains
    the richest stable selection seen in that interval. It never manufactures
    candidates or relaxes the temporal-consistency checks.
    """

    def __init__(self, required_updates: int) -> None:
        if required_updates < 0:
            raise ValueError("required_updates cannot be negative")
        self._required_updates = required_updates
        self._first_sequence: int | None = None
        self._best: PerceivedSelection | None = None

    def observe(
        self, selection: PerceivedSelection, sequence: int
    ) -> PerceivedSelection | None:
        """Return the richest selection once enough new frames arrived."""
        if sequence < 0:
            raise ValueError("sequence cannot be negative")
        if not selection.stable_candidates:
            raise ValueError("selection must contain a stable candidate")
        if self._first_sequence is None:
            self._first_sequence = sequence
        elif sequence < self._first_sequence:
            raise ValueError("sequence cannot move backwards")

        if (
            self._best is None
            or len(selection.stable_candidates)
            >= len(self._best.stable_candidates)
        ):
            self._best = selection

        collected_updates = sequence - self._first_sequence
        if collected_updates >= self._required_updates:
            return self._best
        return None


def merge_perceived_candidates(
    candidate_sets: tuple[tuple[PerceivedPart, ...], ...],
    *,
    maximum_distance: float = 0.04,
) -> tuple[PerceivedPart, ...]:
    """Fuse independent base-frame views without counting a part twice.

    Every detector already transforms depth into ``base_link``.  This layer
    therefore performs only spatial association: observations within one
    workpiece diameter form a cluster whose median rejects view-specific
    edge noise.  Empty views remain valid evidence and cannot invent parts.
    """
    if maximum_distance <= 0.0:
        raise ValueError("maximum_distance must be positive")

    clusters: list[list[PerceivedPart]] = []
    for candidates in candidate_sets:
        for candidate in candidates:
            matching_cluster = next(
                (
                    cluster
                    for cluster in clusters
                    if math.dist(
                        (candidate.x, candidate.y, candidate.z),
                        (
                            median(item.x for item in cluster),
                            median(item.y for item in cluster),
                            median(item.z for item in cluster),
                        ),
                    )
                    <= maximum_distance
                ),
                None,
            )
            if matching_cluster is None:
                clusters.append([candidate])
            else:
                matching_cluster.append(candidate)
    return tuple(
        PerceivedPart(
            median(part.x for part in cluster),
            median(part.y for part in cluster),
            median(part.z for part in cluster),
        )
        for cluster in clusters
    )

def nearest_candidate(
    candidates: tuple[PerceivedPart, ...],
    nominal: tuple[float, float, float],
    *,
    maximum_horizontal_distance: float,
) -> PerceivedPart | None:
    """Select one candidate without assigning semantic identity to all parts."""
    if maximum_horizontal_distance <= 0.0:
        raise ValueError("maximum_horizontal_distance must be positive")
    if not candidates:
        return None
    selected = min(
        candidates,
        key=lambda candidate: math.hypot(
            candidate.x - nominal[0], candidate.y - nominal[1]
        ),
    )
    distance = math.hypot(selected.x - nominal[0], selected.y - nominal[1])
    return selected if distance <= maximum_horizontal_distance else None


def _nearby_matches(
    observations: tuple[tuple[PerceivedPart, ...], ...],
    reference: PerceivedPart,
    *,
    maximum_distance: float,
) -> tuple[PerceivedPart, ...]:
    """Select at most one nearby detection from each camera frame."""

    matches: list[PerceivedPart] = []
    reference_position = (reference.x, reference.y, reference.z)
    for candidates in observations:
        match = nearest_candidate(
            candidates,
            reference_position,
            maximum_horizontal_distance=maximum_distance,
        )
        if match is not None:
            matches.append(match)
    return tuple(matches)


def _track_candidate(
    observations: tuple[tuple[PerceivedPart, ...], ...],
    seed: PerceivedPart,
    *,
    required_observations: int,
    maximum_jitter: float,
) -> tuple[PerceivedPart, int] | None:
    """Build one spatially consistent track while allowing missed frames."""

    broad_matches = _nearby_matches(
        observations,
        seed,
        maximum_distance=2.0 * maximum_jitter,
    )
    if len(broad_matches) < required_observations:
        return None

    center = PerceivedPart(
        median(candidate.x for candidate in broad_matches),
        median(candidate.y for candidate in broad_matches),
        median(candidate.z for candidate in broad_matches),
    )
    confirmed = _nearby_matches(
        observations,
        center,
        maximum_distance=maximum_jitter,
    )
    if len(confirmed) < required_observations:
        return ()
    return (
        PerceivedPart(
            median(candidate.x for candidate in confirmed),
            median(candidate.y for candidate in confirmed),
            median(candidate.z for candidate in confirmed),
        ),
        len(confirmed),
    )


def stable_candidates(
    observations: tuple[tuple[PerceivedPart, ...], ...],
    nominal: tuple[float, float, float],
    *,
    maximum_horizontal_distance: float,
    required_observations: int,
    maximum_jitter: float,
) -> tuple[PerceivedPart, ...]:
    """Return every fresh, stable hypothesis in deterministic order.

    A depth detector may briefly miss one of several otherwise separated
    workpieces. We therefore track individual spatial hypotheses over a short
    window. Requiring the chosen target in the newest frame prevents a stale
    observation from authorizing motion after that target disappears.
    """
    if required_observations < 1:
        raise ValueError("required_observations must be positive")
    if maximum_jitter <= 0.0:
        raise ValueError("maximum_jitter must be positive")
    if len(observations) < required_observations:
        return None

    window_size = required_observations * 2
    recent = observations[-window_size:]
    latest_candidates = tuple(
        candidate
        for candidate in recent[-1]
        if math.hypot(candidate.x - nominal[0], candidate.y - nominal[1])
        <= maximum_horizontal_distance
    )
    hypotheses: list[tuple[PerceivedPart, int]] = []
    for seed in latest_candidates:
        tracked = _track_candidate(
            recent,
            seed,
            required_observations=required_observations,
            maximum_jitter=maximum_jitter,
        )
        if tracked is None:
            continue
        center, support = tracked
        latest_match = nearest_candidate(
            recent[-1],
            (center.x, center.y, center.z),
            maximum_horizontal_distance=maximum_jitter,
        )
        if latest_match is not None:
            hypotheses.append((center, support))

    if not hypotheses:
        return ()
    ordered = sorted(
        hypotheses,
        key=lambda hypothesis: (
            math.hypot(
                hypothesis[0].x - nominal[0],
                hypothesis[0].y - nominal[1],
            ),
            -hypothesis[1],
        ),
    )
    return tuple(center for center, _ in ordered)


def stable_nearest_candidate(
    observations: tuple[tuple[PerceivedPart, ...], ...],
    nominal: tuple[float, float, float],
    *,
    maximum_horizontal_distance: float,
    required_observations: int,
    maximum_jitter: float,
) -> PerceivedPart | None:
    """Return the policy-nearest member of the stable candidate set."""
    candidates = stable_candidates(
        observations,
        nominal,
        maximum_horizontal_distance=maximum_horizontal_distance,
        required_observations=required_observations,
        maximum_jitter=maximum_jitter,
    )
    return candidates[0] if candidates else None


def fresh_candidate_observations(
    history: tuple[
        tuple[float, tuple[PerceivedPart, ...]], ...
    ],
    *,
    requested_at: float,
    now: float,
    maximum_age: float,
    history_window: float,
) -> tuple[tuple[PerceivedPart, ...], ...]:
    """Keep a multi-frame track while requiring a fresh newest frame.

    Camera callbacks can arrive below their simulated rate when Gazebo runs
    slower than real time. A longer consensus window must not make the final
    authorization stale, so history retention and newest-frame freshness are
    deliberately separate limits.
    """
    if maximum_age <= 0.0:
        raise ValueError("maximum_age must be positive")
    if history_window < maximum_age:
        raise ValueError(
            "history_window must be at least as large as maximum_age"
        )
    samples = tuple(
        (received_at, candidates)
        for received_at, candidates in history
        if requested_at <= received_at
        and now - received_at <= history_window
    )
    if not samples or now - samples[-1][0] > maximum_age:
        return ()
    return tuple(candidates for _, candidates in samples)


class RawPartPerception:
    """Cache fresh candidates and reject one-frame RGB-D artifacts."""

    def __init__(
        self,
        node: Node,
        *,
        topic: str = "/perception/raw_part_candidates",
        maximum_age: float = 1.0,
        additional_topics: tuple[str, ...] = (),
        history_window: float = 4.0,
        required_observations: int = 3,
        maximum_jitter: float = 0.015,
        candidate_collection_updates: int = 2,
    ) -> None:
        if maximum_age <= 0.0:
            raise ValueError("maximum_age must be positive")
        if history_window < maximum_age:
            raise ValueError(
                "history_window must be at least as large as maximum_age"
            )
        if required_observations < 1:
            raise ValueError("required_observations must be positive")
        if candidate_collection_updates < 0:
            raise ValueError(
                "candidate_collection_updates cannot be negative"
            )
        topics = (topic, *additional_topics)
        if len(set(topics)) != len(topics):
            raise ValueError("raw-part perception topics must be unique")
        if any(not current.startswith("/") for current in topics):
            raise ValueError("raw-part perception topics must be absolute")
        self._topics = topics
        self._fusion_maximum_distance = 0.04
        self._latest_sources: dict[
            str, tuple[float, tuple[PerceivedPart, ...]]
        ] = {}
        if maximum_jitter <= 0.0:
            raise ValueError("maximum_jitter must be positive")
        self._maximum_age = maximum_age
        self._history_window = history_window
        self._required_observations = required_observations
        self._maximum_jitter = maximum_jitter
        self._candidate_collection_updates = candidate_collection_updates
        self._observation_sequence = 0
        self._lock = threading.Lock()
        self._history: deque[
            tuple[float, tuple[PerceivedPart, ...]]
        ] = deque(maxlen=max(8, required_observations * 2))
        self._subscriptions = [
            node.create_subscription(
                PoseArray,
                current,
                lambda message, source=current: self._remember(
                    source, message
                ),
                10,
            )
            for current in topics
        ]

    def wait_for_selection(
        self,
        selection_reference: tuple[float, float, float],
        *,
        maximum_horizontal_distance: float,
        timeout_sec: float,
    ) -> PerceivedSelection:
        """Select one stable candidate from the complete fresh observation.

        ``selection_reference`` expresses a deterministic robot policy (for
        example, shortest reach from the centre of the accessible bin). It is
        not a taught part position and is never associated with a physical
        entity ID. The returned candidate set lets MoveIt retain every
        non-selected workpiece as collision geometry.
        """
        if timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be positive")
        # A pick request must be based on frames captured after that request.
        # Reusing the docking-time history can mix moving-base observations
        # with the first stationary frames and falsely fail the jitter gate.
        with self._lock:
            requested_at = time.monotonic()
            # No camera may authorize this pick with a pre-request frame.
            self._history.clear()
            self._latest_sources.clear()
        deadline = time.monotonic() + timeout_sec
        observations: tuple[tuple[PerceivedPart, ...], ...] = ()
        collector = StableCandidateCollector(
            self._candidate_collection_updates
        )
        while time.monotonic() < deadline:
            now = time.monotonic()
            with self._lock:
                observations = fresh_candidate_observations(
                    tuple(self._history),
                    requested_at=requested_at,
                    now=now,
                    maximum_age=self._maximum_age,
                    history_window=self._history_window,
                )
                observation_sequence = self._observation_sequence
            stable = stable_candidates(
                observations,
                selection_reference,
                maximum_horizontal_distance=maximum_horizontal_distance,
                required_observations=self._required_observations,
                maximum_jitter=self._maximum_jitter,
            )
            if stable:
                current = PerceivedSelection(
                    target=stable[0],
                    candidates=observations[-1],
                    stable_candidates=stable,
                )
                complete = collector.observe(
                    current, observation_sequence
                )
                if complete is not None:
                    return complete
            time.sleep(0.05)
        candidate_counts = [len(candidates) for candidates in observations]
        raise RuntimeError(
            "no stable, fresh and reachable raw-part candidate was perceived; "
            f"fresh frame candidate counts={candidate_counts}"
        )

    def wait_for_nearest(
        self,
        nominal: tuple[float, float, float],
        *,
        maximum_horizontal_distance: float,
        timeout_sec: float,
    ) -> PerceivedPart:
        """Compatibility wrapper for single-target fixture perception."""

        return self.wait_for_selection(
            nominal,
            maximum_horizontal_distance=maximum_horizontal_distance,
            timeout_sec=timeout_sec,
        ).target

    def _remember(self, source: str, message: PoseArray) -> None:
        candidates = tuple(
            PerceivedPart(
                pose.position.x, pose.position.y, pose.position.z
            )
            for pose in message.poses
        )
        now = time.monotonic()
        with self._lock:
            self._latest_sources[source] = (now, candidates)
            fresh_sources = tuple(
                source_candidates
                for received_at, source_candidates in self._latest_sources.values()
                if now - received_at <= self._maximum_age
            )
            fused = merge_perceived_candidates(
                fresh_sources,
                maximum_distance=self._fusion_maximum_distance,
            )
            self._history.append((now, fused))
            self._observation_sequence += 1
