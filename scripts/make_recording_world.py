#!/usr/bin/env python3
"""Create a recording-only Gazebo world without changing test physics."""

from __future__ import annotations

import argparse
from pathlib import Path


MODEL_START = '<model name="demo_overview_camera">'
INACTIVE = "<always_on>false</always_on>"
ACTIVE = "<always_on>true</always_on>"


def enable_overview_camera(source: Path, destination: Path) -> None:
    text = source.read_text(encoding="utf-8")
    start = text.find(MODEL_START)
    if start < 0:
        raise ValueError("demo_overview_camera model is missing")
    end = text.find("</model>", start)
    if end < 0:
        raise ValueError("demo_overview_camera model is incomplete")
    end += len("</model>")
    model = text[start:end]
    if model.count(INACTIVE) != 1:
        raise ValueError("overview camera must contain one inactive sensor")
    rendered = text[:start] + model.replace(INACTIVE, ACTIVE) + text[end:]
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(rendered, encoding="utf-8")
    temporary.replace(destination)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    enable_overview_camera(args.source, args.destination)


if __name__ == "__main__":
    main()
