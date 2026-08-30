from __future__ import annotations

import re
from typing import List

from pydantic import BaseModel, Field, validator


MACHINE_PATTERN = re.compile(r"^machine_[1-3]$")


class ProductionIntent(BaseModel):
    quantity: int = Field(..., ge=1)
    allowed_machine_ids: List[str] = Field(default_factory=lambda: [
        "machine_1", "machine_2", "machine_3"
    ])
    auto_recharge: bool = True

    @validator("allowed_machine_ids")
    def validate_machines(cls, value):
        deduplicated = list(dict.fromkeys(value))
        if not deduplicated:
            raise ValueError("at least one machine is required")
        invalid = [item for item in deduplicated if not MACHINE_PATTERN.fullmatch(item)]
        if invalid:
            raise ValueError(f"invalid machine ids: {invalid}")
        return deduplicated


ALLOWED_AGENT_TOOLS = frozenset({
    "get_factory_state",
    "submit_order",
    "get_task_status",
    "pause_task",
    "resume_task",
    "cancel_task",
    "explain_failure",
})
