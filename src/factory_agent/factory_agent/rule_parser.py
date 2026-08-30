from __future__ import annotations

import re

from .models import ProductionIntent


class ClarificationRequired(ValueError):
    pass


CHINESE_NUMBERS = {"一": 1, "二": 2, "两": 2, "三": 3, "四": 4, "五": 5, "六": 6}


class ChineseRuleParser:
    """Small, auditable fallback for the canonical production command."""

    def parse(self, text: str) -> ProductionIntent:
        normalized = text.strip().lower()
        if not normalized:
            raise ClarificationRequired("请输入需要加工的零件数量")
        if not any(token in normalized for token in ("加工", "上料", "生产")):
            raise ValueError("规则模式只接受加工、上料或生产订单")

        quantity = self._quantity(normalized)
        machines = [f"machine_{value}" for value in re.findall(r"([1-3])\s*号", normalized)]
        machines += [f"machine_{CHINESE_NUMBERS[value]}" for value in re.findall(
            r"([一二三])号", normalized
        )]
        machines = list(dict.fromkeys(machines)) or ["machine_1", "machine_2", "machine_3"]
        auto_recharge = not any(token in normalized for token in ("不要充电", "禁止充电", "不自动充电"))
        return ProductionIntent(
            quantity=quantity,
            allowed_machine_ids=machines,
            auto_recharge=auto_recharge,
        )

    @staticmethod
    def _quantity(text: str) -> int:
        match = re.search(r"([0-9]+)\s*(?:个|件|枚)", text)
        if match:
            return int(match.group(1))
        match = re.search(r"([一二两三四五六])\s*(?:个|件|枚)", text)
        if match:
            return CHINESE_NUMBERS[match.group(1)]
        raise ClarificationRequired("请明确数量，例如“加工3个零件”")
