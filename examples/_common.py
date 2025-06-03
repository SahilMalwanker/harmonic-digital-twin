"""Shared set-up for the example scripts: paths, plot style and a compact GIF writer."""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib as mpl  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
try:
    import harmonictwin  # noqa: F401
except ImportError:  # running from a fresh clone without `pip install -e .`
    sys.path.insert(0, str(ROOT / "src"))

MEDIA = ROOT / "docs" / "media"
DATA = ROOT / "data"
MEDIA.mkdir(parents=True, exist_ok=True)

PALETTE = {
    "ink": "#1e293b",
    "muted": "#64748b",
    "grid": "#e2e8f0",
    "teal": "#0f766e",
    "orange": "#ea580c",
    "blue": "#2563eb",
    "purple": "#7c3aed",
    "red": "#dc2626",
    "gold": "#ca8a04",
    "green": "#16a34a",
    "measured": "#94a3b8",
}

mpl.rcParams.update(
    {
        "axes.edgecolor": PALETTE["muted"],
        "axes.labelcolor": PALETTE["ink"],
        "axes.titleweight": "bold",
        "axes.titlesize": 10.5,
        "axes.labelsize": 9.5,
        "axes.grid": True,
        "grid.color": PALETTE["grid"],
        "axes.spines.top": False,
        "axes.spines.right": False,
        "xtick.color": PALETTE["muted"],
        "ytick.color": PALETTE["muted"],
        "xtick.labelsize": 8.5,
        "ytick.labelsize": 8.5,
        "legend.frameon": False,
        "legend.fontsize": 8.5,
        "savefig.bbox": "tight",
        "savefig.dpi": 150,
    }
)


def save(fig, name: str) -> Path:
    path = MEDIA / name
    fig.savefig(path)
    print(f"wrote {path.relative_to(ROOT)}")
    return path


def save_gif(fig, update, frames, name: str, fps: float, dpi: int = 80, colors: int = 64) -> Path:
    """Render an animation to a compact GIF: one shared palette, no dithering."""
    from PIL import Image

    fig.set_dpi(dpi)
    shots = []
    for k in frames:
        update(k)
        fig.canvas.draw()
        shots.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()))
    sample = [shots[0], shots[len(shots) // 2], shots[-1]]
    montage = Image.new("RGB", (sample[0].width, sample[0].height * len(sample)))
    for i, im in enumerate(sample):
        montage.paste(im, (0, i * im.height))
    palette = montage.quantize(colors=colors, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)
    quantised = [im.quantize(palette=palette, dither=Image.Dither.NONE) for im in shots]
    path = MEDIA / name
    quantised[0].save(path, save_all=True, append_images=quantised[1:], duration=int(round(1000 / fps)), loop=0, optimize=True)
    print(f"wrote {path.relative_to(ROOT)} ({path.stat().st_size / 1e6:.1f} MB, {len(shots)} frames)")
    return path
