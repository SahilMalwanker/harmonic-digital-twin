# Data

## `src/harmonictwin/data/measured_control_deviation.csv`

Control deviation $e = q_d - q$ in rad, recorded on the single-joint rig at $T_A = 1$ ms for experiments 4–9
(12 252 samples each). Load it with `harmonictwin.load_measurements()`.

| column | controller | model mass in the HMI |
| --- | --- | --- |
| `exp4` | P-PI, $K_L = 7.34$, $K_P = 22.5$, $T_N = 0.144$ s | 11 kg |
| `exp5` | same | 16 kg |
| `exp6` | same | 0 kg |
| `exp7` | P-ReDuS, $K_L = 7.34$, $\alpha = 35$, $\beta = 0$, $K_I = 625$ | 11 kg |
| `exp8` | same | 16 kg |
| `exp9` | same | 0 kg |

All runs use the ramp $v_m = 1.5$ rad/s, $b_m = 1$ rad/s², $s_e = 10$ rad, three plates mounted on one side and
the model inertia $J = 5.4$ kg m². The values were read out of the MATLAB figures exported by the rig's HMI
with [`tools/extract_fig_data.py`](../tools/extract_fig_data.py).

## `hmi_plots/`

Experiments 1–3 (pure model-based control) were only saved as images. These are the HMI's exports of the control
deviation and the control voltage; `examples/pure_model_based.py` places them next to the twin's prediction.
