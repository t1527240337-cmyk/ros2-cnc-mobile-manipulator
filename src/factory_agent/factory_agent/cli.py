from __future__ import annotations

import argparse
import json

from .command_interpreter import AgentCommandInterpreter


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Parse a constrained Chinese factory command"
    )
    parser.add_argument("text")
    args = parser.parse_args()
    result = AgentCommandInterpreter().interpret(args.text)
    payload = json.loads(result.command.json())
    payload["parser"] = result.parser
    payload["knowledge_ids"] = list(result.knowledge_ids)
    payload["fallback_reason"] = result.fallback_reason
    print(json.dumps(payload, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
