Ok — here is a **complete, consistent 2-layer method** where

* **Layer-1 residual = tire residual only** (physically meaningful, constrained),
* **Layer-2 = NN for “everything else”** (biases, unmodeled couplings, low-speed artifacts, discretization, etc.),
* and it works even if you **measure (v_x) but not (v_y)**.

I’ll also tell you what is “best for you” depending on your measurement set, and how LMI / EKF / Luenberger fit in.

---

# 0) First: what you can/can’t identify with “(v_x) only”

If you truly only measure (v_x) + GPS positions (X,Y) (and maybe yaw (\psi)), **(v_y) is not directly observable** unless you have **one more lateral-related piece of info**, typically:

* yaw rate (r) (gyro), and/or
* lateral acceleration (a_y) (IMU), and/or
* GPS velocity vector (course/heading or (V_x,V_y) in global frame).

✅ In practice on vehicles you almost always have IMU (gyro + accel).
So the realistic measurement set is:

[
y_k = [v_x,; r,; \psi,; X,; Y,; a_x,; a_y] \quad (\text{common})
]

If you *really* only have (v_x, X, Y) and nothing else, you can still estimate (v_y) but it becomes fragile (needs differentiating GPS positions → noisy + delay). So I’ll give the clean method assuming you have at least **gyro (r)** and preferably **IMU (a_y)**.

---

# 1) Model split: “tire-only” vs “everything else”

Use discrete-time form:

[
x_{k+1}
=======

f_{\text{nom}}(x_k,u_k)

* B_{\text{tire}}, w_k
* G, d_k
  ]

### Choose what each term means

## Layer-1 unknown (w_k) (tire-only residual)

Make (w_k) represent only **tire force residuals**, e.g.

* (w_k = [\Delta F_{yf}, \Delta F_{yr}]^\top) (lateral forces)
  or if you also want longitudinal tire uncertainty:
* (w_k = [\Delta F_{xf}, \Delta F_{xr}, \Delta F_{yf}, \Delta F_{yr}]^\top)

Then **(B_{\text{tire}})** is fixed and physically derived from Newton–Euler equations.

## Layer-2 unknown (d_k) (everything else)

Let (d_k) represent **non-tire uncertainties**, like:

* IMU bias / gyro bias (slow drift),
* discretization/time delay effects,
* low-speed singularities ((1/v_x) coupling),
* yaw wrap artifacts (state representation problem),
* unmodeled aero/grade/load transfer effects, etc.

Pick (G) so it injects into the channels you mentioned:
[
d_k \text{ affects } \dot v_x,\dot v_y,\dot r
]
(so in discrete time it adds to the corresponding components of (x_{k+1}).)

**Critical condition (to avoid double counting):**
Layer-1 must not be able to explain what layer-2 explains:
[
\text{Range}(B_{\text{tire}})\ \cap\ \text{Range}(G)\ \approx {0}
]
Practically: don’t let (w_k) be a free “add anything to (\dot v_x,\dot v_y,\dot r)” bucket unless it is strictly tied to tire forces.

---

# 2) Layer-1 estimator (physics + tire residual only)

You have 3 mainstream choices:

## A) EKF / UKF augmented with random walk (most practical)

Augment state:
[
\chi_k = \begin{bmatrix} x_k \ w_k \end{bmatrix},
\quad w_{k+1}=w_k+\eta_k
]

Prediction:
[
\hat x^{(1)}_{k+1|k}
====================

f_{\text{nom}}(\hat x^{(1)}*{k|k},u_k)
+
B*{\text{tire}}\hat w^{(1)}*{k|k}
]
[
\hat w^{(1)}*{k+1|k}=\hat w^{(1)}_{k|k}
]

Update with measurement model (y_k=h(x_k)+v_k) (can include IMU accel).

Why good for you: nonlinear vehicle model + you already think Kalman style.

## B) qLPV Luenberger with LMI gains (stability-guaranteed)

Linearize / schedule:
[
x_{k+1}=A(\rho_k)x_k+B(\rho_k)u_k+E(\rho_k) w_k
]
Observer:
[
\hat x_{k+1}=A(\rho_k)\hat x_k+B(\rho_k)u_k+E(\rho_k)\hat w_k
+L(\rho_k)\big(y_k-C(\rho_k)\hat x_k\big)
]
with (L(\rho_k)) from LMI at vertices, scheduled by (\rho_k).

Why good: provable convergence bounds, nice for papers.

## C) MHE (best accuracy, heavier compute)

If you can afford it, MHE gives best robustness with constraints.

---

# 3) Measurements when you have (v_x) but not (v_y)

### Best practice measurement model

Use IMU + GPS to make (v_y) observable **without directly measuring it**:

* GPS gives (X,Y) (positions) and often also speed/course (if available).
* IMU gives (r) and (a_y).

A very effective trick: include **IMU accelerations as measurements** by augmenting state with (a_x,a_y) (your previous 8D “constant C” idea) OR by using nonlinear measurement (h(x,u)) that predicts IMU signals.

That makes (v_y) and tire forces identifiable much faster.

If you refuse to use IMU acceleration, then you rely on GPS differentiation → noisy and delayed → layer-2 will learn junk.

**So for “best for you”: include IMU gyro + accelerations in layer-1 measurement model.**

---

# 4) Layer-2 NN: learn only “leftover mismatch”

This is the key part you asked earlier.

### 4.1 Maintain TWO trajectories: Teacher vs Student

* **Teacher = layer-1 only** (never uses NN).
* **Student = layer-2 corrected** (uses NN in prediction).

**All NN labels come from Teacher only.** That prevents leakage.

### 4.2 Compute the residual label (r_k) at time (k+1)

After teacher has produced (\hat x^{(1)}*{k|k}) and (\hat x^{(1)}*{k+1|k+1}):

Teacher one-step prediction (no NN):
[
\hat x^{(1)}_{k+1|k}
====================

f_{\text{nom}}(\hat x^{(1)}*{k|k},u_k)
+
B*{\text{tire}},\hat w^{(1)}_{k|k}
]

**Residual:**
[
\boxed{
r_k
===

## \hat x^{(1)}_{k+1|k+1}

\hat x^{(1)}_{k+1|k}
}
]

This (r_k) is “what teacher could not explain by nominal + tire residual”.

### 4.3 Turn (r_k) into a target for the NN

You said you want (d) in ((\dot v_x,\dot v_y,\dot r)). Then only train on those components:

Let (S) select those indices:
[
r^{\text{dyn}}_k = S,r_k
]

Now you have two choices:

**Option 1 (explicit (d_k))** if you have an injection matrix (G):
[
r^{\text{dyn}}_k \approx G_d, d_k
\quad\Rightarrow\quad
y^{\text{nn}}_k := G_d^\dagger r^{\text{dyn}}_k
]

**Option 2 (direct residual learning)** (often best):
[
\text{NN}(z_k)\approx r^{\text{dyn}}_k
]
No pseudoinverse, less noise amplification.

### 4.4 What is the NN input (z_k)?

Use signals that explain systematic mismatch:
[
z_k = [\hat x^{(1)}_{k|k},\ u_k,\ \rho_k,\ \text{mode flags}]
]
Include:

* (v_x) (low speed is important),
* slip angle related terms,
* yaw wrap indicator,
* GPS quality flag / innovation norm,
* time step (\Delta t) if variable.

### 4.5 Train only when label is trustworthy (gating)

Do **not** train near bad conditions:

* (v_x < v_{\min}),
* yaw wrap moment,
* GPS jump,
* huge innovation (outlier).

Use a weight:
[
\alpha_k \in [0,1]
]
and minimize:
[
\min_\theta \sum_k \alpha_k |\text{NN}_\theta(z_k)-r^{\text{dyn}}_k|^2
]

---

# 5) Student (layer-2 corrected) prediction/update

Now use NN to improve prediction:

[
\hat r^{\text{dyn}}_k = \text{NN}(z_k)
]

Inject only into those dynamic channels:
[
\hat x^{(2)}_{k+1|k}
====================

f_{\text{nom}}(\hat x^{(2)}*{k|k},u_k)
+
B*{\text{tire}}\hat w^{(1)}_{k|k}
+
S^\top \hat r^{\text{dyn}}_k
]
(meaning: add it only to the chosen state components)

Then optionally do a measurement correction (like EKF update) to keep consistency with measurements:
[
\hat x^{(2)}_{k+1|k+1}
======================

\hat x^{(2)}*{k+1|k} + L^{(2)}*{k+1}\big(y_{k+1}-h(\hat x^{(2)}_{k+1|k})\big)
]

**Important:** do not feed (\hat x^{(2)}) back into teacher.

---

# 6) Which method is best for you (EKF vs LMI vs Luenberger)?

Given your context (vehicle nonlinearities, low speed issues, sensor mix), the most robust and “real” pipeline is:

### ✅ Recommended for you

**Layer-1: EKF (augmented with tire residual random walk) + IMU accel & gyro in measurement**
**Layer-2: NN learns leftover residual in ((v_x,v_y,r)) channels, with gating**

Why:

* EKF handles nonlinear (f(\cdot)) naturally.
* LMI/qLPV is great for theory, but you’ll fight with scheduling/vertices and the true model nonlinearities anyway.
* Your hardest problems (low (v_x), yaw wrap) are not purely “linear gain” problems; they are conditioning + representation issues.

### When to use LMI/Luenberger instead

Use qLPV+LMI if:

* you want a **provable** stability statement in the thesis/paper,
* you have a good polytopic embedding of (A(\rho)),
* and your measurement matrix (C(\rho)) is well-defined across modes.

---

# 7) One crucial note: yaw wrap problem should be fixed at representation level

If your yaw (\psi) jumps (-\pi \leftrightarrow \pi), any learning or observer will suffer.

Best fix: represent yaw as ([\sin\psi,\cos\psi]) in the filter/measurement residual (or at least unwrap angle before innovation).
This alone can reduce the “weird residual” that layer-2 would otherwise learn.

---

## Summary (the full method in one view)

1. **Teacher (Layer-1)** estimates ((x,w_{\text{tire}})) with EKF/MHE/UIO and random walk for (w).
2. After time (k+1) update, compute:
   [
   r_k=\hat x^{(1)}*{k+1|k+1}-\hat x^{(1)}*{k+1|k}
   ]
3. Keep only dynamic channels and train NN on (r_k) (or (G^\dagger r_k)), with gating.
4. **Student (Layer-2)** uses NN to correct prediction, but never contaminates teacher.

---

If you tell me exactly your measurement vector (do you have (r)? (a_y)? (\psi)?), I can write the **exact (h(x,u))** you should use in layer-1 so (v_y) becomes observable and the tire residual stays “tire only” (not a garbage bucket).
