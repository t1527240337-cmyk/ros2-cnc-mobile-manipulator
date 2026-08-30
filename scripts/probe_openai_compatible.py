#!/usr/bin/env python3
"""Probe an OpenAI-compatible provider without printing credentials."""

from __future__ import annotations

import argparse
import json
import os
import urllib.error
import urllib.request


def request_json(url: str, api_key: str, timeout: float) -> dict:
    request = urllib.request.Request(
        url,
        headers={"Authorization": f"Bearer {api_key}"},
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-url", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()

    api_key = os.getenv("OPENAI_API_KEY", "")
    if not api_key:
        raise SystemExit("OPENAI_API_KEY is not configured")

    try:
        document = request_json(
            f"{args.base_url.rstrip('/')}/models",
            api_key,
            args.timeout,
        )
    except (OSError, urllib.error.URLError, json.JSONDecodeError) as error:
        raise SystemExit(f"provider probe failed: {error}") from error

    model_ids = {
        item.get("id")
        for item in document.get("data", [])
        if isinstance(item, dict)
    }
    print(
        json.dumps(
            {
                "authorized": True,
                "model": args.model,
                "model_available": args.model in model_ids,
                "model_count": len(model_ids),
            },
            ensure_ascii=False,
        )
    )
    return 0 if args.model in model_ids else 1


if __name__ == "__main__":
    raise SystemExit(main())
