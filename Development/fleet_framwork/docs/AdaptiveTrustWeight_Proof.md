# Mathematical Proof: Row-Stochastic Property Preservation

**Document:** Formal proof that adaptive weight calculation preserves row-stochastic property  
**Date:** October 5, 2025  
**Status:** Theoretical Foundation

---

## Theorem Statement

**Theorem 1 (Row-Stochastic Preservation):**

Let the weight calculation algorithm compute weights \(W = [w_0, w_1, \ldots, w_N]\) according to the 3-stage process (gating, scoring, normalization). Then:

```math
\sum_{j=0}^{N} w_j = 1 \quad \text{and} \quad w_j \geq 0 \quad \forall j \in \{0, 1, \ldots, N\}
```

**Proof:**

We prove this by construction, showing each stage preserves non-negativity and the normalization stage ensures the sum equals 1.

---

## Stage 1: Gating (Preserves Non-negativity)

**Input:** Trust scores \(t_{\text{loc}}[k], t_{\text{glob}}[k] \in [0,1]\), quality metrics \(\Delta t[k], r[k], \varepsilon[k] \geq 0\).

**Output:** Score \(s[k] \geq 0\) for each neighbor \(k\).

### Hard Gates

For each neighbor \(k\):

```math
\text{If } \Delta t[k] > \Delta t_{\max} \text{ or } \varepsilon[k] > \varepsilon_{\max}: \quad s[k] = 0
```

Since \(s[k] = 0 \geq 0\), non-negativity holds. ✓

### Soft Gates (Hysteresis Taper)

For neighbors not hard-gated:

```math
\alpha_{\text{taper}}[k] = \begin{cases}
1.0 & \text{if trust}[k] > \tau_{\text{high}} \\
0.1 & \text{if trust}[k] < \tau_{\text{low}} \\
\frac{\text{trust}[k] - \tau_{\text{low}}}{\tau_{\text{high}} - \tau_{\text{low}}} & \text{otherwise}
\end{cases}
```

**Analysis:**
- Case 1: \(\alpha_{\text{taper}} = 1.0 \geq 0\) ✓
- Case 2: \(\alpha_{\text{taper}} = 0.1 \geq 0\) ✓
- Case 3: Since \(\text{trust}[k] \in [\tau_{\text{low}}, \tau_{\text{high}}]\), numerator and denominator are both positive, thus \(\alpha_{\text{taper}} \in [0, 1] \geq 0\) ✓

---

## Stage 2: Scoring (Preserves Non-negativity)

For each neighbor \(k\) not hard-gated:

```math
s[k] = \alpha_{\text{taper}}[k] \cdot s_{\text{trust}}[k] \cdot s_{\text{age}}[k] \cdot s_{\text{reliab}}[k] \cdot s_{\text{unc}}[k] \cdot s_{\text{geom}}[k]
```

### Component Analysis

**Trust component:**
```math
s_{\text{trust}}[k] = t_{\text{loc}}[k]^p \cdot t_{\text{glob}}[k]^q
```
- Since \(t_{\text{loc}}, t_{\text{glob}} \in [0, 1]\) and \(p, q > 0\), we have \(s_{\text{trust}} \geq 0\). ✓

**Freshness component:**
```math
s_{\text{age}}[k] = \exp(-\lambda_{\text{age}} \cdot \Delta t[k])
```
- Since \(\exp(x) > 0\) for all \(x \in \mathbb{R}\), we have \(s_{\text{age}} > 0\). ✓

**Reliability component:**
```math
s_{\text{reliab}}[k] = \frac{1}{1 + r[k]}
```
- Since \(r[k] \geq 0\), denominator \(1 + r[k] \geq 1 > 0\), thus \(s_{\text{reliab}} > 0\). ✓

**Uncertainty component:**
```math
s_{\text{unc}}[k] = \frac{1}{1 + c_P \cdot \text{tr}(P_k)}
```
- Covariance trace \(\text{tr}(P_k) \geq 0\) (covariance matrices are positive semi-definite), thus denominator \(> 0\), and \(s_{\text{unc}} > 0\). ✓

**Geometry component:**
```math
s_{\text{geom}}[k] = \frac{1}{1 + c_d \cdot d[k]}
```
- Distance \(d[k] \geq 0\), thus denominator \(> 0\), and \(s_{\text{geom}} > 0\). ✓

**Product of non-negative terms:**

Since \(\alpha_{\text{taper}}[k] \geq 0\) and all components \(s_{\cdot}[k] \geq 0\), their product:

```math
s[k] = \alpha_{\text{taper}}[k] \cdot \prod_{\text{components}} s_{\cdot}[k] \geq 0 \quad \forall k
```

Thus, Stage 2 preserves non-negativity. ✓

---

## Stage 3: Normalization (Ensures Sum = 1)

### Step 3a: Virtual Weight

```math
\alpha_{\text{local}} = \sigma\left(\alpha_0 + \alpha_1 \cdot \text{clamp}(\text{tr}(P_{\text{self}}), [P_{\text{low}}, P_{\text{high}}])\right)
```

Where sigmoid \(\sigma(x) = \frac{1}{1 + e^{-x}} \in (0, 1)\).

```math
w_0 = \text{clamp}(\alpha_{\text{local}}, w_{0,\min}, w_{0,\max})
```

**Bounds:** \(w_0 \in [w_{0,\min}, w_{0,\max}] \subset [0, 1]\).

**Non-negativity:** \(w_0 \geq w_{0,\min} > 0\). ✓

### Step 3b: Self Weight

```math
w_{\text{self,base}} = w_{\text{self,min}} + \frac{\beta_{\text{self}}}{1 + c_{\text{self}} \cdot \text{tr}(P_{\text{self}})}
```

- Since \(\text{tr}(P_{\text{self}}) \geq 0\), denominator \(> 0\), thus \(w_{\text{self,base}} \geq w_{\text{self,min}} > 0\). ✓

**Critical constraint:**
```math
w_{\text{self}} = \text{clamp}(w_{\text{self,base}}, w_{\text{self,min}}, 1 - w_0)
```

**Claim:** \(w_0 + w_{\text{self}} \leq 1\).

**Proof of Claim:**

By the clamp upper bound:
```math
w_{\text{self}} \leq 1 - w_0
```

Therefore:
```math
w_0 + w_{\text{self}} \leq w_0 + (1 - w_0) = 1 \quad \square
```

**Non-negativity:** \(w_{\text{self}} \geq w_{\text{self,min}} > 0\). ✓

### Step 3c: Neighbor Mass

Define:
```math
m_{\text{neighbors}} := 1 - w_0 - w_{\text{self}}
```

**Claim:** \(m_{\text{neighbors}} \geq 0\).

**Proof:** From Step 3b claim, \(w_0 + w_{\text{self}} \leq 1\), thus:
```math
m_{\text{neighbors}} = 1 - w_0 - w_{\text{self}} \geq 0 \quad \square
```

### Step 3d: Neighbor Weight Distribution

Let \(S_{\text{total}} = \sum_{k \in \text{neighbors}} s[k]\) be the sum of neighbor scores.

**Case 1:** \(S_{\text{total}} > 0\) (at least one neighbor has non-zero score).

For each neighbor \(k\):
```math
w_{k+1} = m_{\text{neighbors}} \cdot \frac{s[k]}{S_{\text{total}}}
```

**Non-negativity:** Since \(m_{\text{neighbors}} \geq 0\), \(s[k] \geq 0\), and \(S_{\text{total}} > 0\), we have \(w_{k+1} \geq 0\). ✓

**Sum of neighbor weights:**
```math
\sum_{k \in \text{neighbors}} w_{k+1} = \sum_{k} m_{\text{neighbors}} \cdot \frac{s[k]}{S_{\text{total}}} = m_{\text{neighbors}} \cdot \frac{\sum_k s[k]}{S_{\text{total}}} = m_{\text{neighbors}} \cdot \frac{S_{\text{total}}}{S_{\text{total}}} = m_{\text{neighbors}}
```

**Case 2:** \(S_{\text{total}} = 0\) (all neighbors gated).

Set \(w_{k+1} = 0\) for all neighbors. Then:
```math
\sum_{k \in \text{neighbors}} w_{k+1} = 0
```

In this case, redistribute \(m_{\text{neighbors}}\) to \(w_0\) and \(w_{\text{self}}\) proportionally to maintain sum = 1:
```math
w_0 \leftarrow w_0 + \frac{w_0}{w_0 + w_{\text{self}}} \cdot m_{\text{neighbors}}
```
```math
w_{\text{self}} \leftarrow w_{\text{self}} + \frac{w_{\text{self}}}{w_0 + w_{\text{self}}} \cdot m_{\text{neighbors}}
```

**Verification:**
```math
w_0' + w_{\text{self}}' = (w_0 + w_{\text{self}}) + m_{\text{neighbors}} = 1
```

### Step 3e: Total Weight Sum

**Final weight vector:** \(W = [w_0, w_1, \ldots, w_N]\) where:
- \(w_0\): virtual weight
- \(w_{i+1}\): self weight (vehicle \(i\))
- \(w_{k+1}\): neighbor \(k\) weight for \(k \neq i\)

**Sum:**
```math
\sum_{j=0}^{N} w_j = w_0 + w_{\text{self}} + \sum_{k \in \text{neighbors}} w_{k+1}
```

Substitute from Step 3d:
```math
= w_0 + w_{\text{self}} + m_{\text{neighbors}}
```

Substitute definition of \(m_{\text{neighbors}}\):
```math
= w_0 + w_{\text{self}} + (1 - w_0 - w_{\text{self}}) = 1 \quad \square
```

**Thus, the sum equals 1 exactly.** ✓

---

## Corollary 1 (EMA Smoothing Preserves Row-Stochastic)

**Corollary:** If \(W(t-1)\) and \(W^*(t)\) are both row-stochastic, then:
```math
W(t) = (1 - \eta) W(t-1) + \eta W^*(t)
```
is also row-stochastic.

**Proof:**

**Non-negativity:**
Since \(\eta \in (0, 1)\), \(W(t-1) \geq 0\), and \(W^*(t) \geq 0\), we have:
```math
W(t) = (1-\eta) W(t-1) + \eta W^*(t) \geq 0 \quad \text{(weighted sum of non-negative)}
```

**Sum = 1:**
```math
\sum_j w_j(t) = \sum_j [(1-\eta) w_j(t-1) + \eta w_j^*(t)]
```
```math
= (1-\eta) \sum_j w_j(t-1) + \eta \sum_j w_j^*(t)
```
```math
= (1-\eta) \cdot 1 + \eta \cdot 1 = 1 \quad \square
```

---

## Corollary 2 (Bounded Estimation Error)

**Corollary (Boundedness Lemma):**

If \(W\) is row-stochastic with \(w_j \geq 0\), then for any state vector \(x(t)\):
```math
\|x(t+1)\|_\infty \leq \|x(t)\|_\infty
```

**Proof:**

Let \(x_i(t+1) = \sum_j w_{ij} x_j(t)\).

Since \(W\) is row-stochastic (\(\sum_j w_{ij} = 1\)) and \(w_{ij} \geq 0\), the weighted sum is a **convex combination**:
```math
\min_j x_j(t) \leq x_i(t+1) \leq \max_j x_j(t)
```

Therefore:
```math
|x_i(t+1)| \leq \max_j |x_j(t)| = \|x(t)\|_\infty
```

Taking max over all \(i\):
```math
\|x(t+1)\|_\infty = \max_i |x_i(t+1)| \leq \|x(t)\|_\infty \quad \square
```

**Implication:** State estimates never explode; the system is **numerically stable**.

---

## Theorem 2 (Consensus Convergence)

**Theorem (Consensus under Connectivity):**

Let \(G = (V, E)\) be a **strongly connected** graph with \(N\) vehicles. Let each vehicle \(i\) compute weights \(W_i\) according to the adaptive algorithm. If:

1. **Aperiodicity:** \(w_{\text{self}}^{(i)} \geq w_{\min} > 0\) for all \(i\),
2. **Bounded influence:** \(\max_{k \neq i} w_k^{(i)} \leq \alpha < 1\),
3. **Row-stochastic:** \(\sum_j w_j^{(i)} = 1\), \(w_j^{(i)} \geq 0\),

Then the distributed observer converges to consensus:
```math
\lim_{t \to \infty} \|x̂_i(t) - x̂_j(t)\| = 0 \quad \forall i, j \in V
```

**Proof Sketch:**

This is a standard result in consensus theory (Jadbabaie et al., 2003). The key steps:

1. **Construct global weight matrix:** \(\mathbf{W} \in \mathbb{R}^{N \times N}\) where \(\mathbf{W}_{ij} = w_j^{(i)}\).

2. **Row-stochastic property:** By Theorem 1, each row of \(\mathbf{W}\) sums to 1.

3. **Aperiodicity + connectivity:** Ensures \(\mathbf{W}\) is **primitive** (exists \(k\) such that \(\mathbf{W}^k\) has all positive entries).

4. **Perron-Frobenius Theorem:** For primitive stochastic matrices, the eigenvalue \(\lambda = 1\) is simple, and all other eigenvalues satisfy \(|\lambda_i| < 1\).

5. **Convergence:** The iteration \(x(t+1) = \mathbf{W} x(t)\) converges to the **consensus subspace** spanned by the eigenvector of \(\lambda = 1\). \(\square\)

**Reference:** A. Jadbabaie, J. Lin, and A. S. Morse, "Coordination of groups of mobile autonomous agents using nearest neighbor rules," IEEE TAC, 2003.

---

## Summary

**Main Results:**

1. ✅ **Row-stochastic preservation** (Theorem 1): The 3-stage algorithm ensures \(\sum w_j = 1\) and \(w_j \geq 0\) by construction.

2. ✅ **EMA smoothing compatibility** (Corollary 1): Smoothing preserves row-stochastic property.

3. ✅ **Numerical stability** (Corollary 2): Bounded estimation error (\(\|x(t)\|_\infty\) non-increasing).

4. ✅ **Consensus convergence** (Theorem 2): Under connectivity and aperiodicity, fleet reaches consensus.

**Implementation Guarantee:**

As long as the implementation follows the normalization procedure in Stage 3 (specifically, the constraint \(w_{\text{self}} \leq 1 - w_0\) and the definition \(m_{\text{neighbors}} = 1 - w_0 - w_{\text{self}}\)), the weights will **always** be row-stochastic, ensuring stable and convergent distributed estimation.

---

**End of Proof**

*For implementation details, see `AdaptiveTrustWeightDesign.md` and `AdaptiveTrustWeight_Clarifications.md`.*
