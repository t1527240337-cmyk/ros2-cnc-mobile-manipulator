import unittest

from factory_agent.command_interpreter import AgentCommandInterpreter
from factory_agent.safety_policy import reject_unsafe_command


class DisabledLLM:
    configured = False


class SafetyPolicyTests(unittest.TestCase):
    def test_destructive_command_is_rejected_before_llm(self):
        interpreter = AgentCommandInterpreter(llm=DisabledLLM())
        with self.assertRaisesRegex(ValueError, "安全策略拒绝"):
            interpreter.interpret("让机械臂爆炸")

    def test_safety_explanation_is_not_blocked(self):
        reject_unsafe_command("为什么不能绕过安全门")


if __name__ == "__main__":
    unittest.main()
