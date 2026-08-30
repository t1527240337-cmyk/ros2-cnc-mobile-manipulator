import unittest

from pydantic import ValidationError

from factory_agent.interpreter import AgentInterpreter
from factory_agent.models import ALLOWED_AGENT_TOOLS, ProductionIntent
from factory_agent.rule_parser import ChineseRuleParser, ClarificationRequired


class DisabledLLM:
    configured = False


class BrokenLLM:
    configured = True

    def parse_intent(self, _text):
        raise ValueError("malformed model output")


class RuleParserTests(unittest.TestCase):
    def test_chinese_quantity(self):
        intent = ChineseRuleParser().parse("将三个毛坯分配给空闲机床，加工后放入成品框")
        self.assertEqual(intent.quantity, 3)
        self.assertEqual(len(intent.allowed_machine_ids), 3)

    def test_selected_machines(self):
        intent = ChineseRuleParser().parse("使用1号和3号机床加工2个零件")
        self.assertEqual(intent.quantity, 2)
        self.assertEqual(intent.allowed_machine_ids, ["machine_1", "machine_3"])

    def test_missing_quantity_requires_clarification(self):
        with self.assertRaises(ClarificationRequired):
            ChineseRuleParser().parse("帮我加工零件")

    def test_non_positive_quantity_rejected(self):
        with self.assertRaises(ValidationError):
            ProductionIntent(quantity=0)

    def test_rule_parser_leaves_live_capacity_to_factory_state(self):
        intent = ChineseRuleParser().parse("加工4个零件")
        self.assertEqual(intent.quantity, 4)

    def test_invalid_machine_rejected(self):
        with self.assertRaises(ValidationError):
            ProductionIntent(quantity=1, allowed_machine_ids=["machine_9"])


class InterpreterTests(unittest.TestCase):
    def test_no_api_uses_rules(self):
        result = AgentInterpreter(llm=DisabledLLM()).interpret("加工3个零件")
        self.assertEqual(result.parser, "rules")

    def test_bad_llm_falls_back(self):
        result = AgentInterpreter(llm=BrokenLLM()).interpret("加工2个零件")
        self.assertEqual(result.parser, "rules")
        self.assertIn("malformed", result.fallback_reason)

    def test_tool_whitelist_has_no_low_level_controls(self):
        forbidden = {"cmd_vel", "send_joint_trajectory", "machine_command", "dock_robot"}
        self.assertTrue(ALLOWED_AGENT_TOOLS.isdisjoint(forbidden))


if __name__ == "__main__":
    unittest.main()
