"""Regenerate every figure and animation in docs/media."""

from __future__ import annotations

import runpy
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCRIPTS = [
    "controller_design.py",
    "identification.py",
    "twin_vs_measured.py",
    "pure_model_based.py",
    "robustness.py",
    "closing_the_gap.py",
    "rig_animation.py",
]

if __name__ == "__main__":
    sys.path.insert(0, str(HERE))
    for name in SCRIPTS:
        print(f"== {name}")
        runpy.run_path(str(HERE / name), run_name="__main__")
