import json
import os
import threading
import unittest
import urllib.error
import urllib.request

from factory_agent.operator_web import (
    FactoryOperatorHttpServer,
    OperatorWebApplication,
    OperatorWebConfig,
)


class OperatorWebTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        async def chat_handler(message):
            return {
                "reply": f"收到：{message}",
                "knowledge_ids": ["authority_boundary"],
                "tool_executions": [{
                    "name": "get_factory_state",
                    "protocol_succeeded": True,
                    "payload": {
                        "accepted": True,
                        "data": {"machines": []},
                    },
                }],
            }

        config = OperatorWebConfig(
            mcp_url="http://127.0.0.1:8000/mcp",
            model="test-model",
            llm_base_url="http://127.0.0.1:18081/v1",
        )
        cls.server = FactoryOperatorHttpServer(
            ("127.0.0.1", 0),
            OperatorWebApplication(config, chat_handler),
            quiet=True,
        )
        cls.thread = threading.Thread(
            target=cls.server.serve_forever,
            daemon=True,
        )
        cls.thread.start()
        host, port = cls.server.server_address
        cls.base_url = f"http://{host}:{port}"

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=2)

    def test_serves_product_ui_with_security_headers(self):
        with urllib.request.urlopen(
            f"{self.base_url}/",
            timeout=2,
        ) as response:
            body = response.read().decode("utf-8")
            headers = response.headers

        self.assertIn("移动机械臂操作台", body)
        self.assertIn("default-src 'self'", headers["Content-Security-Policy"])
        self.assertEqual(headers["X-Frame-Options"], "DENY")
        self.assertEqual(headers["X-Content-Type-Options"], "nosniff")

    def test_health_does_not_expose_api_key(self):
        previous_key = os.environ.get("OPENAI_API_KEY")
        os.environ["OPENAI_API_KEY"] = "must-not-leak"
        try:
            payload = self._get_json("/api/health")
        finally:
            if previous_key is None:
                del os.environ["OPENAI_API_KEY"]
            else:
                os.environ["OPENAI_API_KEY"] = previous_key

        self.assertTrue(payload["llm_configured"])
        self.assertEqual(payload["model"], "test-model")
        self.assertNotIn("must-not-leak", json.dumps(payload))

    def test_chat_returns_structured_tool_trace(self):
        payload = self._post_json(
            "/api/chat",
            {"message": "查看工厂状态"},
        )

        self.assertEqual(payload["reply"], "收到：查看工厂状态")
        execution = payload["tool_executions"][0]
        self.assertEqual(execution["name"], "get_factory_state")
        self.assertTrue(execution["payload"]["accepted"])

    def test_empty_chat_is_rejected(self):
        with self.assertRaises(urllib.error.HTTPError) as raised:
            self._post_json("/api/chat", {"message": "   "})
        self.assertEqual(raised.exception.code, 400)
        payload = json.loads(
            raised.exception.read().decode("utf-8")
        )
        self.assertIn("empty", payload["error"])

    def test_unknown_route_is_not_implicitly_served(self):
        with self.assertRaises(urllib.error.HTTPError) as raised:
            urllib.request.urlopen(
                f"{self.base_url}/private",
                timeout=2,
            )
        self.assertEqual(raised.exception.code, 404)

    def _get_json(self, path):
        with urllib.request.urlopen(
            f"{self.base_url}{path}",
            timeout=2,
        ) as response:
            return json.loads(response.read().decode("utf-8"))

    def _post_json(self, path, payload):
        request = urllib.request.Request(
            f"{self.base_url}{path}",
            data=json.dumps(
                payload,
                ensure_ascii=False,
            ).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=2) as response:
            return json.loads(response.read().decode("utf-8"))


if __name__ == "__main__":
    unittest.main()
