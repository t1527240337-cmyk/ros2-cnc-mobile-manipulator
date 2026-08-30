from __future__ import annotations

from dataclasses import dataclass

from .command_models import AgentCommand
from .command_parser import ChineseCommandParser
from .knowledge import KnowledgeBase
from .llm_client import OpenAICompatibleClient
from .safety_policy import reject_unsafe_command


@dataclass(frozen=True)
class CommandInterpretation:
    command: AgentCommand
    parser: str
    knowledge_ids: tuple[str, ...]
    fallback_reason: str = ""


class AgentCommandInterpreter:
    def __init__(
        self,
        llm: OpenAICompatibleClient | None = None,
        rules: ChineseCommandParser | None = None,
        knowledge: KnowledgeBase | None = None,
    ):
        self.llm = llm or OpenAICompatibleClient()
        self.rules = rules or ChineseCommandParser()
        self.knowledge = knowledge or KnowledgeBase()

    def interpret(self, text: str) -> CommandInterpretation:
        reject_unsafe_command(text)
        entries = self.knowledge.retrieve(text)
        knowledge_ids = tuple(entry.entry_id for entry in entries)
        context = self.knowledge.format_context(entries)

        if self.llm.configured:
            try:
                command = self.llm.parse_command(text, context)
                return CommandInterpretation(command, "llm+rag", knowledge_ids)
            except (RuntimeError, ValueError) as exc:
                command = self.rules.parse(text)
                return CommandInterpretation(
                    command, "rules", knowledge_ids, str(exc)
                )

        command = self.rules.parse(text)
        return CommandInterpretation(
            command,
            "rules",
            knowledge_ids,
            "LLM is not configured",
        )
