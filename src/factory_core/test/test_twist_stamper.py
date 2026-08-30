import pytest

from factory_core.twist_stamper import VelocitySourceArbiter


def test_navigation_is_available_before_precision_control():
    arbiter = VelocitySourceArbiter(hold_sec=0.2)
    assert arbiter.accepts_navigation_command(now=10.0)


def test_precision_command_temporarily_blocks_navigation():
    arbiter = VelocitySourceArbiter(hold_sec=0.2)
    arbiter.register_precision_command(now=10.0)
    assert not arbiter.accepts_navigation_command(now=10.19)
    assert arbiter.accepts_navigation_command(now=10.20)


def test_each_precision_command_renews_the_lease():
    arbiter = VelocitySourceArbiter(hold_sec=0.2)
    arbiter.register_precision_command(now=10.0)
    arbiter.register_precision_command(now=10.15)
    assert not arbiter.accepts_navigation_command(now=10.30)
    assert arbiter.accepts_navigation_command(now=10.35)


def test_hold_time_must_be_positive():
    with pytest.raises(ValueError, match="hold_sec must be positive"):
        VelocitySourceArbiter(hold_sec=0.0)
