# MATLAB originals

The MATLAB files written for the lab's preparation tasks, kept as they were for reference (only the author
fields in the Simulink file's metadata were cleared). The maintained, tested implementation is the Python
package in [`src/harmonictwin`](../src/harmonictwin).

| File | Task | Purpose |
| --- | --- | --- |
| `Task_4_1.m` | 4.1 | $M^*$ and $b^*(q,\dot q) = k_1\dot q + k_2\cos q$ from the rig parameters. |
| `Task_4_2.m` | 4.2 | $a_0, a_1$ of the desired PT2 and $r_0$, $U_S$ at one instant. |
| `Task_4_3.m` | 4.3 | PI velocity controller, step response from the Simulink model, PT2 approximation and $K_L$. |
| `Task_4_4.m` | 4.4 | Times $t_b, t_v, t_e$ of the ramp profile. |
| `Cascaded_Control_System.slx` | 4.3 | Simulink model of the PI velocity loop on the virtual plant (Step → PI → $1/s$), MATLAB R2024a. |

**Requirements:** MATLAB, plus Simulink and the Control System Toolbox (`stepinfo`) for `Task_4_3.m`.
Run `Cascaded_Control_System.slx` first; `Task_4_3.m` reads its `out` variable.

## Notes from the port

- `Task_4_1.m` labels `u` as a voltage; it is the gear ratio (160).
- `Task_4_3.m` reads overshoot and rise time from the Simulink output. Depending on the solver's output
  points that gives 15.1 % and 84 ms, hence $K_L = 7.34\ \text{s}^{-1}$, the value used on the rig. The exact step
  response (`harmonictwin.step_response`) has 15.5 % and 82.8 ms, which gives $K_L = 7.39\ \text{s}^{-1}$.
