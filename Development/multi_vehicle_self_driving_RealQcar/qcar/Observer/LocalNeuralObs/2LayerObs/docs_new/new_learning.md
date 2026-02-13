

## Definitions

Discrete-time nominal model (what you already have):
[
x_{k+1}=f_{\text{nom}}(x_k,u_k)+B_{\text{tire}},w_k+G,d_k
]

* Layer 1 estimates (x) and (w) (tire residual), **ignoring (d)** in its model.
* Layer 2 NN learns (d) (or learns state residual directly).

We will compute:
[
r_k := \hat x^{(1)}*{k+1|k+1}-\Big(f*{\text{nom}}(\hat x^{(1)}*{k|k},u_k)+B*{\text{tire}}\hat w^{(1)}_{k|k}\Big)
]
**Important:** every (\hat{\cdot}^{(1)}) is layer-1 only. No NN anywhere in this computation.

---

# Exact timeline (where to compute (r_k))

### At time (k): after you receive (y_k)

**Layer 1 update (UIO/KF/MHE):**

1. Predict with layer-1 model (no NN):
   [
   \hat x^{(1)}*{k|k-1}=f*{\text{nom}}(\hat x^{(1)}*{k-1|k-1},u*{k-1})+B_{\text{tire}}\hat w^{(1)}*{k-1|k-1}
   ]
   [
   \hat w^{(1)}*{k|k-1}=\hat w^{(1)}_{k-1|k-1} \quad (\text{random walk})
   ]

2. Correct using measurements (GPS on/off mode etc.):
   [
   \hat x^{(1)}*{k|k}=\hat x^{(1)}*{k|k-1}+L_k\Big(y_k-h(\hat x^{(1)}_{k|k-1})\Big)
   ]
   (and whatever update for (\hat w^{(1)}) your filter uses)

✅ **Stop here.** At time (k) you do **not** yet have (r_k) (because it needs (\hat x^{(1)}_{k+1|k+1})).

---

### At time (k+1): after you receive (y_{k+1})

Repeat the layer-1 predict+correct:

[
\hat x^{(1)}*{k+1|k}=f*{\text{nom}}(\hat x^{(1)}*{k|k},u_k)+B*{\text{tire}}\hat w^{(1)}*{k|k}
]
[
\hat x^{(1)}*{k+1|k+1}=\hat x^{(1)}*{k+1|k}+L*{k+1}\Big(y_{k+1}-h(\hat x^{(1)}_{k+1|k})\Big)
]

### **Now compute the residual (r_k)**

This is the exact “no leak” spot:

[
\boxed{
r_k
:= \hat x^{(1)}*{k+1|k+1}
-\Big(f*{\text{nom}}(\hat x^{(1)}*{k|k},u_k)+B*{\text{tire}}\hat w^{(1)}_{k|k}\Big)
}
]

That’s it.

* RHS uses **layer-1** state at time (k) and layer-1 tire residual at time (k).
* LHS uses **layer-1** corrected state at time (k+1).
* NN never appears → no label leakage.

---

## How to form the NN label from (r_k)

### Option A (learn (d_k) explicitly)

If you have a known injection matrix (G) for “general disturbance in (\dot v_x,\dot v_y,\dot r)” (or their discrete equivalents), then:

[
\hat y^{\text{nn}}_k := G^\dagger r_k
]

Better (less noisy) is ridge:
[
G^\dagger_\lambda=(G^\top G+\lambda I)^{-1}G^\top,\quad
\hat y^{\text{nn}}*k = G^\dagger*\lambda r_k
]

### Option B (learn residual in state channels directly)

Train NN to output (\hat r_k\approx r_k), then inject it as an additive correction in prediction.

---

# How layer 2 is used without leaking into labels

You asked: **“That layer 2 that have a updated model?”**

Think of layer 2 as an *add-on term* to the predictor **used for control / downstream**, but not used to generate the training labels.

### The “augmented predictor” used online (after NN trained a bit)

Use:
[
\hat d_k = \text{NN}(z_k)
]
and predict a “better” state:
[
\hat x^{(2)}_{k+1|k}
====================

f_{\text{nom}}(\hat x^{(2)}*{k|k},u_k)
+B*{\text{tire}}\hat w^{(1)}_{k|k}
+G,\hat d_k
]

Then you can do a measurement correction for layer 2 as well:
[
\hat x^{(2)}*{k+1|k+1}
=\hat x^{(2)}*{k+1|k}+L^{(2)}*{k+1}(y*{k+1}-h(\hat x^{(2)}_{k+1|k}))
]

✅ **But labels still come from layer 1 only**:

* (\hat x^{(1)}) and (\hat w^{(1)}) produce (r_k).
* layer 2 uses NN for better tracking/control, **not** for label generation.

This is the clean separation.

---

## The simplest “no leak” implementation rule

Maintain two parallel states:

* **Teacher (layer 1):** ((\hat x^{(1)},\hat w^{(1)})) — *never uses NN*
* **Student (layer 2):** (\hat x^{(2)}) — uses NN in prediction

Then:

* Train NN using residuals computed from **teacher only**:
  [
  r_k = \hat x^{(1)}*{k+1|k+1} - \hat x^{(1)}*{k+1|k}
  ]
  (note: (\hat x^{(1)}*{k+1|k} = f*{\text{nom}}(\hat x^{(1)}*{k|k},u_k)+B*{\text{tire}}\hat w^{(1)}_{k|k}))

This also makes it super easy in code: teacher filter runs first, then residual computed, then NN updated, then student predictor runs.

---

## One extra detail (important): don’t let the teacher’s state depend on student

If you ever “replace” (\hat x^{(1)}) with (\hat x^{(2)}) because it looks better, you reintroduce leakage.

So: teacher is “boring but stable”; student can be fancy.

---

If you want, I can write it as compact pseudo-code (loop over k) exactly matching your 8D state and the disturbance channels ((\dot v_x,\dot v_y,\dot r)) so you see precisely what (G) and (r_k) look like in discrete time.
