import unittest
from types import SimpleNamespace

from lifecycle_msgs.msg import State

from factory_core.dock_station import DockStationClient


class _CompletedFuture:
    def __init__(self, state_id):
        self._response = SimpleNamespace(
            current_state=SimpleNamespace(id=state_id)
        )

    def done(self):
        return True

    def result(self):
        return self._response


class _LifecycleClient:
    def __init__(self, state_ids):
        self._state_ids = iter(state_ids)
        self.requests = 0

    def wait_for_service(self, timeout_sec):
        return timeout_sec > 0.0

    def call_async(self, _request):
        self.requests += 1
        return _CompletedFuture(next(self._state_ids))


class DockLifecycleReadinessTests(unittest.TestCase):
    def test_waits_past_inactive_action_discovery_until_active(self):
        client = DockStationClient.__new__(DockStationClient)
        client._lifecycle = _LifecycleClient(
            [
                State.PRIMARY_STATE_INACTIVE,
                State.PRIMARY_STATE_ACTIVE,
            ]
        )
        client._callback_wait = lambda: None

        self.assertTrue(client.wait_until_active(timeout_sec=1.0))
        self.assertEqual(client._lifecycle.requests, 2)

    def test_missing_lifecycle_service_is_not_ready(self):
        client = DockStationClient.__new__(DockStationClient)
        client._lifecycle = SimpleNamespace(
            wait_for_service=lambda timeout_sec: False
        )

        self.assertFalse(client.wait_until_active(timeout_sec=0.01))


if __name__ == "__main__":
    unittest.main()
