"""Select a confirmed empty finished-bin slot from typed RGB-D observations."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading
import time

from factory_interfaces.msg import TrayOccupancy
from rclpy.node import Node


@dataclass(frozen=True)
class SlotOccupancy:
    """The safety-relevant state of one tray slot in one camera frame."""

    slot_id: int
    observable: bool
    occupied: bool


def choose_stable_empty_slot(
    observations: tuple[dict[int, SlotOccupancy], ...],
    preferred_slots: tuple[int, ...],
    *,
    required_observations: int,
) -> int | None:
    """Return the first preferred slot confirmed empty in every recent frame."""

    if required_observations < 1:
        raise ValueError("required_observations must be positive")
    if not preferred_slots or any(slot_id < 1 for slot_id in preferred_slots):
        raise ValueError("preferred_slots must contain positive slot ids")
    if len(set(preferred_slots)) != len(preferred_slots):
        raise ValueError("preferred_slots cannot contain duplicates")
    if len(observations) < required_observations:
        return None

    recent = observations[-required_observations:]
    for slot_id in preferred_slots:
        states = tuple(frame.get(slot_id) for frame in recent)
        if all(
            state is not None
            and state.observable
            and not state.occupied
            for state in states
        ):
            return slot_id
    return None


def slot_has_stable_state(
    observations: tuple[dict[int, SlotOccupancy], ...],
    slot_id: int,
    *,
    occupied: bool,
    required_observations: int,
) -> bool:
    """Require one observable slot to hold one state in every recent frame."""

    if slot_id < 1:
        raise ValueError("slot_id must be positive")
    if required_observations < 1:
        raise ValueError("required_observations must be positive")
    if len(observations) < required_observations:
        return False
    states = tuple(
        frame.get(slot_id) for frame in observations[-required_observations:]
    )
    return all(
        state is not None
        and state.observable
        and state.occupied is occupied
        for state in states
    )


def unreserved_slot_preferences(
    preferred_slots: tuple[int, ...],
    reserved_slots: set[int] | frozenset[int],
) -> tuple[int, ...]:
    """Preserve preference order while excluding completed destinations."""

    if not preferred_slots or any(slot_id < 1 for slot_id in preferred_slots):
        raise ValueError("preferred_slots must contain positive slot ids")
    if len(set(preferred_slots)) != len(preferred_slots):
        raise ValueError("preferred_slots cannot contain duplicates")
    if any(slot_id < 1 for slot_id in reserved_slots):
        raise ValueError("reserved_slots must contain positive slot ids")
    return tuple(
        slot_id
        for slot_id in preferred_slots
        if slot_id not in reserved_slots
    )


def fresh_occupancy_observations(
    history: tuple[tuple[float, dict[int, SlotOccupancy]], ...],
    *,
    requested_at: float,
    now: float,
    maximum_age: float,
) -> tuple[dict[int, SlotOccupancy], ...]:
    """Use only post-request frames and require the newest frame to be fresh."""

    if maximum_age <= 0.0:
        raise ValueError("maximum_age must be positive")
    samples = tuple(
        (received_at, slots)
        for received_at, slots in history
        if received_at >= requested_at and now - received_at <= maximum_age
    )
    if not samples or now - samples[-1][0] > maximum_age:
        return ()
    return tuple(slots for _, slots in samples)


class FinishedSlotPerception:
    """Cache finished-bin occupancy and expose a bounded blocking selection."""

    def __init__(
        self,
        node: Node,
        *,
        topic: str = "/perception/finished_bin_slots",
        tray_id: str = "finished_bin",
        maximum_age: float = 1.0,
        required_observations: int = 3,
    ) -> None:
        if maximum_age <= 0.0:
            raise ValueError("maximum_age must be positive")
        if required_observations < 1:
            raise ValueError("required_observations must be positive")
        self._tray_id = tray_id
        self._maximum_age = maximum_age
        self._required_observations = required_observations
        self._clock = node.get_clock()
        self._lock = threading.Lock()
        self._history: deque[
            tuple[float, dict[int, SlotOccupancy]]
        ] = deque(maxlen=max(8, required_observations * 2))
        self._subscription = node.create_subscription(
            TrayOccupancy, topic, self._remember, 10
        )

    def wait_for_empty(
        self,
        preferred_slots: tuple[int, ...],
        *,
        timeout_sec: float,
        include_recent_history: bool = False,
    ) -> int:
        if timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be positive")
        now = self._now()
        requested_at = (
            now - self._maximum_age
            if include_recent_history
            else now
        )
        deadline = time.monotonic() + timeout_sec
        observations: tuple[dict[int, SlotOccupancy], ...] = ()
        while time.monotonic() < deadline:
            now = self._now()
            with self._lock:
                observations = fresh_occupancy_observations(
                    tuple(self._history),
                    requested_at=requested_at,
                    now=now,
                    maximum_age=self._maximum_age,
                )
            selected = choose_stable_empty_slot(
                observations,
                preferred_slots,
                required_observations=self._required_observations,
            )
            if selected is not None:
                return selected
            time.sleep(0.05)

        raise RuntimeError(
            "no finished-bin slot was observable and empty in "
            f"{self._required_observations} fresh frames"
        )

    def begin_observation_window(self) -> float:
        """Return a ROS-time marker for post-release observations."""

        return self._now()

    def wait_for_occupied(
        self,
        slot_id: int,
        *,
        requested_at: float,
        timeout_sec: float,
    ) -> None:
        """Require a selected slot to become visibly occupied after release."""

        if slot_id < 1:
            raise ValueError("slot_id must be positive")
        if timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be positive")
        deadline = time.monotonic() + timeout_sec
        observations: tuple[dict[int, SlotOccupancy], ...] = ()
        while time.monotonic() < deadline:
            now = self._now()
            with self._lock:
                observations = fresh_occupancy_observations(
                    tuple(self._history),
                    requested_at=requested_at,
                    now=now,
                    maximum_age=self._maximum_age,
                )
            if slot_has_stable_state(
                observations,
                slot_id,
                occupied=True,
                required_observations=self._required_observations,
            ):
                return
            time.sleep(0.05)
        raise RuntimeError(
            f"finished-bin slot {slot_id} was not observable and occupied in "
            f"{self._required_observations} post-release frames"
        )

    def _remember(self, message: TrayOccupancy) -> None:
        if message.tray_id != self._tray_id:
            return
        slots = {
            int(slot.slot_id): SlotOccupancy(
                slot_id=int(slot.slot_id),
                observable=bool(slot.observable),
                occupied=bool(slot.occupied),
            )
            for slot in message.slots
            if int(slot.slot_id) > 0
        }
        with self._lock:
            self._history.append((self._now(), slots))

    def _now(self) -> float:
        """Return ROS time so freshness follows simulation time when enabled."""

        return self._clock.now().nanoseconds * 1.0e-9
