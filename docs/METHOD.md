# Method notes: statics & dynamics parameter identification

Technical summary of the algorithms implemented in this repository.

**中文版：** [`zh/METHOD.md`](zh/METHOD.md)

## Overall pipeline

```text
URDF (demo 7-DoF or custom)
    │
    ├─ statics: slow cosine trajectory
    │     Y = [Y_g (RNEA numeric columns) | Y_fc]
    │     estimate π_g + Coulomb friction  →  gravity compensation
    │
    └─ dynamics: Fourier trajectory (analytic q, dq, ddq)
          Y = [Y_dyn (Pinocchio regressor) | Y_fc | Y_fv]
          estimate base inertial params + friction  →  torque feedforward

Shared post-processing: pivoted QR → OLS / Huber-IRLS / whitened robust WLS
Validation: noise + outlier spikes; inlier RMSE; gravity-compensation check;
            optional trajectory collision playback
```

## Statics vs dynamics

| | Statics | Dynamics |
|--|---------|----------|
| When | Low-speed gravity compensation is enough | Need inertia / high-speed feedforward |
| Trajectory | Slow cosine sweep | Finite Fourier series |
| Estimated | Gravity-related terms + Coulomb friction | Inertia + Coriolis/centrifugal + gravity + Coulomb/viscous |
| Key metric | Gravity compensation error | Base-parameter rank; inlier torque RMSE |

## Causal chain (algorithm rationale)

1. **Goal:** Gravity compensation / torque feedforward needs inertial and friction parameters \(π\), not only PID gains.
2. **Linearity:** Rigid-body torque is affine in inertial parameters: \(τ = Y(q,\dot q,\ddot q)\,π\). Statics (zero velocity/acceleration) keeps gravity + Coulomb friction.
3. **Building \(Y\):**
   - Statics: Pinocchio has no analytic \(Y_g\) → place unit mass / first mass moments per body, run RNEA(\(q,0,0\)).
   - Dynamics: `computeJointTorqueRegressor` + smooth Coulomb + viscous columns.
4. **QR:** Full parameter vectors are not uniquely identifiable → column-pivoted QR yields a base set and improves conditioning.
5. **Robust + Fourier:** Fourier keeps \(q,\dot q,\ddot q\) consistent and band-limited; Huber down-weights large residuals; whitening handles joint heteroscedasticity; hard thresholding rejects spikes.
6. **Use in control:** \(\hatπ\) → \(τ_g = Y_g(q)\hatπ_g\) or \(τ_{ff}=Y(q,\dot q,\ddot q)\hatπ\).

## Interpreting typical logs

| Log item | Meaning |
|----------|---------|
| QR 28→14 (statics) / 84→59 (dynamics) | Full columns compressed to base parameters |
| RMSE (all) ≫ RMSE (inlier) | All-sample RMSE is inflated by injected torque spikes |
| Gravity compensation ~0.01 N·m | Most relevant check for statics quality under simulation |
| Collision view green | Excitation trajectory has no non-adjacent mesh collisions |

## FAQ (technical)

**What does statics estimate?** Gravity-related parameters and Coulomb friction. Enough when the controller mainly needs gravity compensation.

**Why numeric \(Y_g\)?** Pinocchio does not expose an analytic gravity regressor; unit-parameter RNEA columns construct it.

**Why QR?** Over-parameterization → non-unique / ill-conditioned LS solutions.

**OLS vs robust WLS?** OLS is biased by torque spikes; Huber reweights; whitening removes residual correlation/heteroscedasticity; outer loop zeros samples with large whitened residuals.

**Why not random \((q,\dot q,\ddot q)\)?** Samples are kinematically inconsistent and hard to track on hardware; Fourier trajectories are executable with controlled spectrum.

**Simulation vs hardware?** Here torques are synthesized from URDF ground truth + noise. Hardware adds flexibility, temperature, drive dynamics, sensor bias. The pipeline transfers; the numbers do not.

**SDP feasibility?** Optional positive-semidefinite pseudo-inertia constraints (e.g. cvxpy). Not enabled by default; main path is QR + robust WLS.

## Code map

| Module | Role |
|--------|------|
| `param_id/robot_model.py` | URDF resolve + Pinocchio model |
| `param_id/regressor.py` | \(Y_g\), dynamics \(Y\), friction |
| `param_id/base_params.py` | Pivoted QR base columns |
| `param_id/estimators.py` | OLS / Huber / robust WLS |
| `param_id/trajectory.py` | Fourier & cosine |
| `scripts/identify_*.py` | End-to-end identification demos |
| `scripts/collision_view.py` | Trajectory replay + collision check |

## Experiment log (optional)

| Item | Value |
|------|-------|
| Date | ________ |
| Statics OLS gravity-compensation error | ________ |
| Statics robust WLS gravity-compensation error | ________ |
| Parameter change / observation | ________ |
