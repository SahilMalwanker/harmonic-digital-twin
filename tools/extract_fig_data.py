"""Extract the measured control deviation from the rig's exported MATLAB figures into one CSV.

Usage:  python tools/extract_fig_data.py <folder with the .fig files> src/harmonictwin/data/measured_control_deviation.csv

A .fig file is a MATLAB v5 MAT-file holding the handle-graphics tree, so scipy can read the line data
without MATLAB. Requires scipy (not a runtime dependency of the package).
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from scipy.io import loadmat

FILES = {
    4: "control_dev_exp4.fig",
    5: "control_dev_exp5.fig",
    6: "control_dev_exp6.fig",
    7: "Control Deviation exp7.fig",
    8: "Control Deviation exp8.fig",
    9: "Control Deviation exp9.fig",
}


def line_data(node):
    if getattr(node, "type", "") == "graph2d.lineseries":
        props = node.properties
        return np.asarray(props.XData, float), np.asarray(props.YData, float)
    children = getattr(node, "children", None)
    for child in np.atleast_1d(children) if children is not None else []:
        found = line_data(child)
        if found is not None:
            return found
    return None


def main(src: Path, dst: Path) -> None:
    series = {}
    for exp, name in FILES.items():
        path = next(src.rglob(name))
        t, e = line_data(loadmat(path, squeeze_me=True, struct_as_record=False)["hgS_070000"])
        series[exp] = (t, e)
    n = min(len(t) for t, _ in series.values())
    t = series[4][0][:n]
    for exp, (te, _) in series.items():
        if not np.allclose(te[:n], t):
            raise ValueError(f"time base of experiment {exp} differs")
    header = (
        "# Control deviation e = q_d - q [rad] measured on the single-joint rig, sampled at T_A = 1 ms.\n"
        "# Ramp profile v_m = 1.5 rad/s, b_m = 1 rad/s^2, s_e = 10 rad. Model inertia J = 5.4 kg m^2 in all runs.\n"
        "# exp4-6: P-PI cascade, K_L = 7.34, K_P = 22.5, T_N = 0.144 s; model mass m = 11 / 16 / 0 kg.\n"
        "# exp7-9: P-ReDuS cascade, K_L = 7.34, alpha = 35, beta = 0, K_I = 625; model mass m = 11 / 16 / 0 kg.\n"
        "t," + ",".join(f"exp{k}" for k in series)
    )
    table = np.column_stack([t] + [series[k][1][:n] for k in series])
    np.savetxt(dst, table, delimiter=",", header=header, comments="", fmt=["%.3f"] + ["%.6f"] * len(series))
    print(f"wrote {dst} ({n} samples)")


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]))
