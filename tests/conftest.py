"""Shared pytest configuration for the RynnRCP test suite."""

from __future__ import annotations

import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOTS = [
    PROJECT_ROOT,
    PROJECT_ROOT / "apps" / "common",
    PROJECT_ROOT / "apps" / "mcp",
    PROJECT_ROOT / "apps" / "rynnbot",
    PROJECT_ROOT / "apps" / "teleop",
    PROJECT_ROOT / "robots" / "tetheria_aerohand",
    PROJECT_ROOT / "robots" / "lerobot_so101",
    PROJECT_ROOT / "robots" / "lerobot_lekiwi",
]

for source_root in reversed(SOURCE_ROOTS):
    source_path = str(source_root)
    if source_path not in sys.path:
        sys.path.insert(0, source_path)
