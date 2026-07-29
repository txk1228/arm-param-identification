# Pipeline reproduction & module map

Reproduce the full identification flow by stage, and map \(τ = Yπ\), QR base parameters, Huber / whitened WLS, and Fourier excitation to their code locations.

**中文版：** [`zh/LEARNING.md`](zh/LEARNING.md)

---

## 0. Environment

```bash
conda activate param-id   # or env_isaaclab
cd <repo-root>
export PARAM_ID_URDF=$PWD/models/demo_7dof/demo_arm.urdf
```

Layout (full tree: [README.md](../README.md) / [docs/README.md](README.md)):

```
param_id/                 # Algorithm package
utils/                    # Data I/O
scripts/                  # CLI entry points
models/demo_7dof/         # Public educational URDF
assets/proprietary/       # Local private URDF/meshes (gitignored)
docs/                     # Documentation index → docs/README.md
results/                  # Run outputs → results/README.md
```

Private CAD stays under `assets/proprietary/` and must not be committed. Public default: `models/demo_7dof/demo_arm.urdf`.

---

## 1. Reproduction order

Stages follow dependency order: regressor consistency → statics → dynamics → trajectory feasibility → ablations.

### Stage 0 — Regressor consistency

```bash
python scripts/00_sanity_check.py
```

Both residuals should be ~`0`:

| Check | Formula |
|-------|---------|
| Statics | \(τ_g = Y_g(q) π_g\) via numeric RNEA columns |
| Dynamics | \(τ = Y(q,\dot q,\ddot q) π\) via `computeJointTorqueRegressor` |

### Stage 1 — Statics identification

```bash
python scripts/identify_static.py --method ols --outlier-ratio 0.05
python scripts/identify_static.py --method huber --outlier-ratio 0.05
python scripts/identify_static.py --method robust_wls --outlier-ratio 0.05
```

Checks:

1. QR: gravity columns compressed (e.g. 28 → ~12–14)
2. **Gravity compensation error** at end of log (preferable to all-sample RMSE when outliers exist)
3. `results/static_*.png` — red markers are injected spikes

### Stage 2 — Dynamics identification

```bash
python scripts/identify_dynamic.py --method ols --n-periods 3
python scripts/identify_dynamic.py --method robust_wls --outlier-ratio 0.05
```

Checks:

1. Fourier \(q\) traces in `results/dynamic_*.png`
2. Full → base parameter dimension
3. Inlier torque RMSE vs all-sample RMSE
4. True vs estimated base-parameter bars

### Stage 3 — Trajectory collision check

Geometric validation of excitation trajectories (not identification itself).

```bash
python scripts/collision_view.py --traj fourier --check-only
python scripts/collision_view.py --traj cosine --check-only
python scripts/collision_view.py --traj cv --check-only
python scripts/collision_view.py --traj fourier --view
```

Controls: auto-play on open; `SPACE` pause/resume; `N`/`B` step; arrows orbit; `ESC` quit.  
If collisions with `base_link` persist, try `--amplitude-scale 0.4` or `--ignore-base`.

### Stage 4 — Ablations

| Experiment | Change | Expected |
|------------|--------|----------|
| Weaker excitation | smaller `--amplitude-scale` | worse conditioning / larger error |
| Dirtier data | `--outlier-scale 80 --outlier-ratio 0.1` | OLS degrades; robust methods hold up |
| Longer trajectory | `--n-periods 10` | stabler estimates, slower runtime |

---

## 2. Concept ↔ code map

| Concept | Location |
|---------|----------|
| Numeric \(Y_g\) via RNEA | `param_id/regressor.py` → `gravity_regressor_numeric` |
| JointTorqueRegressor + friction | `param_id/regressor.py` → `dynamics_regressor` |
| Pivoted QR base set | `param_id/base_params.py` |
| OLS / Huber / whitened WLS | `param_id/estimators.py` |
| Fourier + outliers | `param_id/trajectory.py` + `scripts/identify_*.py` |
| URDF loading | `param_id/robot_model.py` |

SDP pseudo-inertia constraints (cvxpy) are **not** enabled by default. Main path: QR + robust WLS.

See [`METHOD.md`](METHOD.md) for algorithm rationale.

---

## 3. Scope

- Torques are synthesized from URDF ground-truth parameters, then noise/outliers are added.
- Simulation RMSE is not hardware identification accuracy.
- See also [`METHOD.md`](METHOD.md).
