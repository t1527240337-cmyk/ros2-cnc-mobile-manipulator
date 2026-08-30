import pytest

from factory_core.undock_station import (
    remaining_clearance_after_nav2_undock,
    uses_nav2_undocking,
)


def test_cnc_uses_measured_local_retreat() -> None:
    assert not uses_nav2_undocking("factory_station")


def test_bins_and_charger_keep_nav2_undocking() -> None:
    assert uses_nav2_undocking("factory_bin_station")
    assert uses_nav2_undocking("charging_station")


def test_bin_staging_pose_already_clears_standard_work_envelope() -> None:
    assert remaining_clearance_after_nav2_undock(
        "factory_bin_station", 0.30
    ) == pytest.approx(0.0)


def test_larger_bin_clearance_request_keeps_only_uncovered_distance() -> None:
    assert remaining_clearance_after_nav2_undock(
        "factory_bin_station", 0.40
    ) == pytest.approx(0.05)


def test_charger_keeps_measured_post_undock_clearance() -> None:
    assert remaining_clearance_after_nav2_undock(
        "charging_station", 0.30
    ) == pytest.approx(0.30)


def test_negative_clearance_request_is_rejected() -> None:
    with pytest.raises(ValueError, match="non-negative"):
        remaining_clearance_after_nav2_undock("factory_bin_station", -0.01)
