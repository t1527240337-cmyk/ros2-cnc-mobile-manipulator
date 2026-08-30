from __future__ import annotations

from dataclasses import dataclass
from importlib.resources import files

import yaml


@dataclass(frozen=True)
class KnowledgeEntry:
    entry_id: str
    title: str
    keywords: tuple[str, ...]
    text: str


class KnowledgeBase:
    """Small auditable retrieval layer for SOP and safety knowledge."""

    def __init__(self, entries: list[KnowledgeEntry] | None = None):
        self.entries = entries or self._load_packaged_entries()

    @staticmethod
    def _load_packaged_entries() -> list[KnowledgeEntry]:
        resource = files("factory_agent").joinpath("knowledge_base.yaml")
        data = yaml.safe_load(resource.read_text(encoding="utf-8"))
        return [
            KnowledgeEntry(
                entry_id=str(item["id"]),
                title=str(item["title"]),
                keywords=tuple(str(keyword) for keyword in item.get("keywords", [])),
                text=str(item["text"]).strip(),
            )
            for item in data["entries"]
        ]

    def retrieve(self, query: str, limit: int = 3) -> list[KnowledgeEntry]:
        normalized = query.lower()
        ranked = []
        for entry in self.entries:
            score = sum(keyword.lower() in normalized for keyword in entry.keywords)
            if score:
                ranked.append((score, entry.entry_id, entry))
        ranked.sort(key=lambda item: (-item[0], item[1]))

        selected = [item[2] for item in ranked[:limit]]
        if not selected:
            selected = [self._entry("authority_boundary")]
        elif all(entry.entry_id != "authority_boundary" for entry in selected):
            selected = [self._entry("authority_boundary"), *selected[:limit - 1]]
        return selected

    @staticmethod
    def format_context(entries: list[KnowledgeEntry]) -> str:
        return "\n".join(
            f"[{entry.entry_id}] {entry.title}: {entry.text}" for entry in entries
        )

    def explain(self, query: str) -> str:
        return "；".join(entry.text for entry in self.retrieve(query))

    def _entry(self, entry_id: str) -> KnowledgeEntry:
        return next(entry for entry in self.entries if entry.entry_id == entry_id)
