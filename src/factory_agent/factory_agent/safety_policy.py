from __future__ import annotations


DESTRUCTIVE_TARGETS = (
    "爆炸",
    "破坏设备",
    "撞毁",
    "绕过安全",
    "屏蔽急停",
    "强制开门",
    "直接控制关节",
    "发布cmd_vel",
    "发布 cmd_vel",
)

COMMAND_MARKERS = (
    "让",
    "使",
    "帮我",
    "执行",
    "命令",
    "要求",
    "直接",
)


def reject_unsafe_command(text: str) -> None:
    """Reject destructive actuator requests before either LLM or rule parsing."""

    normalized = text.lower().replace(" ", "")
    destructive = any(target.replace(" ", "") in normalized for target in DESTRUCTIVE_TARGETS)
    commanding = any(marker in normalized for marker in COMMAND_MARKERS)
    if destructive and commanding:
        raise ValueError(
            "请求被安全策略拒绝：Agent 不执行破坏设备、绕过安全互锁或直接控制执行器的命令"
        )
