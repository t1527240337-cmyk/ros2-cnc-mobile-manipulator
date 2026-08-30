import json
import tempfile
import unittest
from pathlib import Path

from factory_agent.audit import CommandAuditLog


class CommandAuditLogTests(unittest.TestCase):
    def test_record_is_append_only_and_machine_readable(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "agent.jsonl"
            audit = CommandAuditLog(path)

            audit.record(
                request_id="request-1",
                source="mcp",
                operation="get_factory_state",
                accepted=True,
                order_id="",
                message="ok",
            )
            audit.record(
                request_id="request-2",
                source="rules",
                operation="submit_order",
                accepted=False,
                order_id="",
                message="rejected",
            )

            entries = [
                json.loads(line)
                for line in path.read_text(encoding="utf-8").splitlines()
            ]

        self.assertEqual(len(entries), 2)
        self.assertEqual(entries[0]["request_id"], "request-1")
        self.assertTrue(entries[0]["accepted"])
        self.assertEqual(entries[1]["operation"], "submit_order")
        self.assertFalse(entries[1]["accepted"])
        self.assertIn("timestamp", entries[0])


if __name__ == "__main__":
    unittest.main()
