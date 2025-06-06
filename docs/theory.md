# Theory notes

## 1. Plant model

Joint 3 of a 6-axis arm is mounted on a test stand: brushless motor with resolver, harmonic drive ($u = 160$), and a link
that can carry plates on one side. With a fast current loop ($I_A = U_S / K_{MI}$), a stiff gear and $K_M = C/K_{MI}$, the
motor and joint equations combine into

$$
(M + J_A u^2)\,\ddot q = U_S K_M u - F_M u^2 \dot q - m g l_s \cos q
\quad\Longleftrightarrow\quad
M^*\ddot q = U_S - b^*(q, \dot q),
$$

$$
M^* = \frac{M + J_A u^2}{K_M u} = 0.3377\ \tfrac{\text{V s}^2}{\text{rad}},\qquad
b^* = \underbrace{\frac{F_M u^2}{K_M u}}_{k_1 = 0.9245}\dot q + \underbrace{\frac{m g l_s}{K_M u}}_{k_2 = 0.8881}\cos q .
$$

The twin adds two things the lab model leaves out: a Coulomb term $U_c\,\mathrm{sign}(\dot q)$ (smoothed) and an angle
offset in the gravity term, $\cos(q + \varphi)$.

## 2. Model-based control

The controller inverts its *believed* model, $U_S = \tilde M^* r_0 + \tilde b^*(q, \dot q)$. If the belief is right, the plant
reduces to the virtual plant $\ddot q = r_0$, a double integrator. Any mismatch enters as an acceleration disturbance

$$
\ddot q = r_0 + d,\qquad d = \frac{\tilde b^* - b^*}{M^*}\quad(\text{for }\tilde M^* = M^*).
$$

**Pure model-based PT2.** $r_0 = a_0(q_d - q) - a_1\dot q$ with $a_0 = 1/T_R^2$, $a_1 = 2d_R/T_R$. Following a ramp of speed
$v$ it lags by $e = a_1 v / a_0$ (0.24 rad at 1.5 rad/s), and without an integrator it keeps any constant disturbance
as a steady-state error. Feeding forward the planned acceleration and velocity,
$r_0 = \ddot q_d + a_1(\dot q_d - \dot q) + a_0(q_d - q)$, turns it into computed-torque control.

**P-PI cascade.** $v^* = K_L e + K_V\dot q_d$, $r_0 = K_P\big(v^* - \dot q\big) + \tfrac{K_P}{T_N}\!\int(v^* - \dot q)$. The velocity loop
$G_v = (1 + T_N s)/(1 + T_N s + \tfrac{T_N}{K_P}s^2)$ is matched to a PT2 with $T_N = 2d_{Rv}T_{Rv}$ and $K_P = T_N/T_{Rv}^2$.

**P-ReDuS cascade.** $r_0 = K_I\!\int(v^* - \dot q) + \beta v^* - \alpha\dot q$. For the virtual plant $1/s$
($a_0 = 0$, $a_1 = 1$, $a_2 = 0$) the design rules give $\beta = 0$, $K_I = 1/T_R^2$, $\alpha = 2 d_R T_R K_I$, so the
velocity loop is *exactly* the desired PT2.

**Position gain.** The rule of thumb $K_L = 0.25/T_v$ uses a PT2 fitted to the velocity loop's step response from its
overshoot and 0–100 % rise time, $d_v = 1/\sqrt{1 + (\pi/\ln OV)^2}$ and $T_v = T_{an}\sqrt{1-d_v^2}/(\pi - \arccos d_v)$.

## 3. Why the cascades lag by 204 mrad

Without velocity pre-control ($K_V = 0$), the velocity set-point is $K_L e$. Cruising at $v$ therefore requires
$e = v/K_L = 1.5/7.34 = 0.2044$ rad, whatever the velocity controller or the model accuracy. This is what all six
recordings show. $K_V = 1$ removes the requirement; adding $\ddot q_d$ (and, for ReDuS, $(\alpha - \beta)\dot q_d$) to $r_0$
lets the loop track with zero controller state.

## 4. Identifying the unbalanced load from the ripple

While cruising, a gravity mismatch produces a disturbance at the rotation frequency $\omega = v$. Writing signals as
phasors over the joint angle, $e = \mathrm{Re}(P e^{iq})$ and

$$
P = H(i\omega)\,\frac{g\,l_s}{K_M u M^*}\,\big(m\,e^{i\varphi} - m_{model}\big) = G\,\big(m\,e^{i\varphi} - m_{model}\big),
$$

where $H$ is the closed-loop response from an acceleration disturbance to the position error, e.g. for the PI cascade
$H(s) = s/\big(s^3 + K_P s^2 + (K_P K_L + K_P/T_N)s + K_P K_L/T_N\big)$.

$P$ is linear in the model mass. Fitting $P_k = a + b\,m_k$ to runs that differ only in $m_k$ gives
$m\,e^{i\varphi} = -a/b$, without knowing $H$ at all. $P_k$ comes from a least-squares fit of
$e = c_0 + c_c\cos q + c_s\sin q$ over the cruise phase, $P = c_c - i c_s$.
Experiments 4–6 (PI) and 7–9 (ReDuS) give $m = 14.34$ and $14.37$ kg, $\varphi = -6.4°$ and $-6.3°$. With these numbers the
analytic $H$ predicts every measured ripple to within 3 % in amplitude and 3° in phase.
