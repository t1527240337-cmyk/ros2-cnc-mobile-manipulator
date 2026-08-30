import unittest

from pydantic import ValidationError

from factory_agent.command_models import (
    FORBIDDEN_LOW_LEVEL_INTERFACES,
    AgentCommand,
    AgentOperation,
)
from factory_agent.command_parser import ChineseCommandParser
from factory_agent.knowledge import KnowledgeBase


class CommandParserTests(unittest.TestCase):
    def setUp(self):
        self.parser = ChineseCommandParser()

    def test_state_query(self):
        command = self.parser.parse("查看机床状态和库存")
        self.assertEqual(command.operation, AgentOperation.GET_FACTORY_STATE)

    def test_machine_hold_and_resume(self):
        hold = self.parser.parse("暂停2号机床加工")
        resume = self.parser.parse("恢复二号机床继续加工")
        self.assertEqual(hold.operation, AgentOperation.HOLD_MACHINE)
        self.assertEqual(hold.target_machine_id, "machine_2")
        self.assertEqual(resume.operation, AgentOperation.RESUME_MACHINE)

    def test_task_cancel_is_not_machine_hold(self):
        command = self.parser.parse("取消任务")
        self.assertEqual(command.operation, AgentOperation.CANCEL_TASK)

    def test_automatic_production_control(self):
        start = self.parser.parse("启动自动生产")
        stop = self.parser.parse("完成当前订单后停止自动生产")
        status = self.parser.parse("查看自动生产状态")

        self.assertEqual(start.operation, AgentOperation.START_AUTOMATIC)
        self.assertEqual(stop.operation, AgentOperation.STOP_AUTOMATIC)
        self.assertEqual(
            status.operation, AgentOperation.GET_AUTOMATIC_STATUS
        )

    def test_automatic_mode_can_be_limited_to_one_machine(self):
        command = self.parser.parse("只用2号机床启动自动生产")

        self.assertEqual(command.operation, AgentOperation.START_AUTOMATIC)
        self.assertEqual(
            command.allowed_machine_ids,
            ["machine_2"],
        )

    def test_automatic_mode_preserves_multiple_machine_preferences(self):
        command = self.parser.parse("使用三号和1号机床启动自动生产")

        self.assertEqual(command.operation, AgentOperation.START_AUTOMATIC)
        self.assertEqual(
            command.allowed_machine_ids,
            ["machine_3", "machine_1"],
        )

    def test_capability_query(self):
        command = self.parser.parse("你能做什么")
        self.assertEqual(command.operation, AgentOperation.LIST_CAPABILITIES)

    def test_low_level_interfaces_are_never_tools(self):
        self.assertIn("/cmd_vel", FORBIDDEN_LOW_LEVEL_INTERFACES)

    def test_explanation_keeps_the_user_query(self):
        command = self.parser.parse("为什么2号机床加工失败")
        self.assertEqual(command.operation, AgentOperation.EXPLAIN_FAILURE)
        self.assertIn("失败", command.query)

    def test_explanation_requires_non_empty_query(self):
        with self.assertRaises(ValidationError):
            AgentCommand(
                operation=AgentOperation.EXPLAIN_FAILURE,
                query="  ",
            )

    def test_task_identifier_is_constrained(self):
        command = AgentCommand(
            operation=AgentOperation.PAUSE_TASK,
            task_id="agent-order.12:retry",
        )
        self.assertEqual(command.task_id, "agent-order.12:retry")
        with self.assertRaises(ValidationError):
            AgentCommand(
                operation=AgentOperation.PAUSE_TASK,
                task_id="../../unsafe task",
            )


class KnowledgeTests(unittest.TestCase):
    def test_stop_query_retrieves_safety_boundary(self):
        entries = KnowledgeBase().retrieve("停止加工和急停有什么区别")
        identifiers = {entry.entry_id for entry in entries}
        self.assertIn("authority_boundary", identifiers)
        self.assertIn("controlled_stop", identifiers)


if __name__ == "__main__":
    unittest.main()
