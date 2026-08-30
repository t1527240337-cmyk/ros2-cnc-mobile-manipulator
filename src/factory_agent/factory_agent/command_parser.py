from __future__ import annotations

import re

from .command_models import AgentCommand, AgentOperation
from .rule_parser import ChineseRuleParser, ClarificationRequired


class ChineseCommandParser:
    """Auditable fallback router for common operator commands."""

    def __init__(self, order_parser: ChineseRuleParser | None = None):
        self.order_parser = order_parser or ChineseRuleParser()

    def parse(self, text: str) -> AgentCommand:
        normalized = text.strip().lower()
        if not normalized:
            raise ClarificationRequired("请输入生产或查询指令")

        machine_ids = self._machine_ids(normalized)
        machine_id = machine_ids[0] if machine_ids else None
        automatic = self._contains(normalized, "自动生产", "自动模式")
        if automatic and self._contains(normalized, "状态", "进度", "情况"):
            return AgentCommand(
                operation=AgentOperation.GET_AUTOMATIC_STATUS
            )
        if automatic and self._contains(
            normalized, "启动", "开始", "开启", "运行"
        ):
            return AgentCommand(
                operation=AgentOperation.START_AUTOMATIC,
                allowed_machine_ids=machine_ids,
            )
        if automatic and self._contains(
            normalized, "停止", "关闭", "结束", "不再"
        ):
            return AgentCommand(operation=AgentOperation.STOP_AUTOMATIC)
        if self._contains(normalized, "能做什么", "有哪些功能", "帮助", "能力"):
            return AgentCommand(operation=AgentOperation.LIST_CAPABILITIES)
        if self._contains(normalized, "为什么", "解释故障", "故障原因", "怎么处理"):
            return AgentCommand(
                operation=AgentOperation.EXPLAIN_FAILURE,
                target_machine_id=machine_id,
                query=normalized,
            )
        if machine_id and self._contains(normalized, "暂停", "停止", "保持", "hold"):
            return AgentCommand(
                operation=AgentOperation.HOLD_MACHINE,
                target_machine_id=machine_id,
            )
        if machine_id and self._contains(normalized, "恢复", "继续"):
            return AgentCommand(
                operation=AgentOperation.RESUME_MACHINE,
                target_machine_id=machine_id,
            )
        if self._contains(normalized, "取消任务", "取消订单", "终止任务", "停止任务"):
            return AgentCommand(operation=AgentOperation.CANCEL_TASK)
        if self._contains(normalized, "暂停任务", "暂停订单"):
            return AgentCommand(operation=AgentOperation.PAUSE_TASK)
        if self._contains(normalized, "恢复任务", "继续任务", "继续订单"):
            return AgentCommand(operation=AgentOperation.RESUME_TASK)
        if self._contains(normalized, "任务状态", "任务进度", "订单进度"):
            return AgentCommand(operation=AgentOperation.GET_TASK_STATUS)
        if self._contains(normalized, "工厂状态", "机床状态", "库存", "电量"):
            return AgentCommand(operation=AgentOperation.GET_FACTORY_STATE)
        if self._contains(normalized, "加工", "上料", "生产"):
            return self._order_command(normalized)
        raise ValueError("无法识别指令；可询问“你能做什么”查看高层功能")

    def _order_command(self, text: str) -> AgentCommand:
        intent = self.order_parser.parse(text)
        return AgentCommand(
            operation=AgentOperation.SUBMIT_ORDER,
            quantity=intent.quantity,
            allowed_machine_ids=intent.allowed_machine_ids,
            auto_recharge=intent.auto_recharge,
        )

    @staticmethod
    def _machine_ids(text: str) -> list[str]:
        """Return mentioned machines once, preserving the operator's order."""

        if "机床" not in text:
            return []

        ordered_ids: list[str] = []
        chinese = {"一": "1", "二": "2", "三": "3"}
        for match in re.finditer(r"([1-3一二三])\s*号", text):
            machine_number = chinese.get(
                match.group(1),
                match.group(1),
            )
            machine_id = f"machine_{machine_number}"
            if machine_id not in ordered_ids:
                ordered_ids.append(machine_id)
        return ordered_ids

    @staticmethod
    def _contains(text: str, *tokens: str) -> bool:
        return any(token in text for token in tokens)
