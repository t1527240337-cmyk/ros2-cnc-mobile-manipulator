from __future__ import annotations

from dataclasses import dataclass

from .llm_client import OpenAICompatibleClient
from .models import ProductionIntent
from .rule_parser import ChineseRuleParser


@dataclass(frozen=True)
class Interpretation:
    intent: ProductionIntent
    parser: str
    fallback_reason: str = ""


class AgentInterpreter:
    def __init__(self, llm: OpenAICompatibleClient | None = None,
                 rules: ChineseRuleParser | None = None):
        self.llm = llm or OpenAICompatibleClient()
        self.rules = rules or ChineseRuleParser()

    def interpret(self, text: str) -> Interpretation:
        if self.llm.configured:
            try:
                return Interpretation(self.llm.parse_intent(text), "llm")
            except (RuntimeError, ValueError) as exc:
                return Interpretation(self.rules.parse(text), "rules", str(exc))
        return Interpretation(self.rules.parse(text), "rules", "LLM is not configured")
