# 7-DoF Arm Dynamics / Statics Parameter Identification

[![Python 3.11](https://img.shields.io/badge/python-3.11-blue.svg)](https://www.python.org/)
[![Pinocchio](https://img.shields.io/badge/Pinocchio-2.x-green.svg)](https://stack-of-tasks.github.io/pinocchio/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Simulation pipeline for **robot inertial & friction parameter identification** on a 7-DoF manipulator:

- **Statics:** gravity + Coulomb friction → gravity compensation  
- **Dynamics:** inertia + Coriolis/centrifugal + gravity + Coulomb/viscous friction → torque feedforward  
- **Robust estimation:** pivoted QR base parameters, OLS / Huber-IRLS / whitened WLS  
- **Excitation:** Fourier series (dynamics) & slow cosine (statics)  
- **Validation:** outlier injection, gravity-compensation check, trajectory collision playback  

> This repository is a **self-contained simulation** of robot parameter identification
> for study and open-source sharing. Joint torques are synthesized from URDF
> ground-truth parameters (plus noise/outliers). Reported errors are **not**
> hardware identification accuracy.

中文说明见 [`docs/zh/README.md`](docs/zh/README.md)。  
方法说明：[`docs/METHOD.md`](docs/METHOD.md) · 操作指南：[`docs/LEARNING.md`](docs/LEARNING.md)

---

## Pipeline

```text
URDF (demo 7-DoF or your arm)
        │
        ├─ statics: cosine trajectory
        │     Y = [Y_g (RNEA numeric) | Y_fc]  →  π_g, π_fc
        │
        └─ dynamics: Fourier trajectory (q, dq, ddq)
              Y = [Y_dyn (Pinocchio) | Y_fc | Y_fv]  →  base inertial + friction
                        │
                        ▼
              pivoted QR → OLS / Huber / robust WLS
                        │
                        ▼
         gravity compensation / torque prediction plots
         (+ optional Trimesh collision playback)
```

---

## Quick start

### 1. Environment

```bash
# Option A: conda (recommended)
conda env create -f environment.yml
conda activate param-id

# Option B: existing env that already has Pinocchio (e.g. env_isaaclab)
conda activate env_isaaclab
pip install -r requirements.txt   # numpy/scipy/matplotlib/trimesh if missing
```

### 2. Run the public demo (no proprietary CAD)

```bash
cd /path/to/param_id

# Force public demo arm (primitives only, ships with the repo)
export PARAM_ID_URDF=$PWD/models/demo_7dof/demo_arm.urdf

python scripts/00_sanity_check.py
python scripts/identify_static.py  --method robust_wls --outlier-ratio 0.05
python scripts/identify_dynamic.py --method robust_wls --outlier-ratio 0.05 --n-periods 3
python scripts/collision_view.py   --traj fourier --check-only
python scripts/collision_view.py   --traj fourier --view   # optional GUI
```

Results are written to `results/`.

### 3. (Optional) Your own URDF

```bash
python scripts/identify_dynamic.py --urdf /path/to/your_arm.urdf --method robust_wls
```

See [`assets/proprietary/README.md`](assets/proprietary/README.md) for local private assets (gitignored).

---

## Repository layout

```text
param_id/                 # Python package (regressors, estimators, trajectories)
scripts/                  # CLI entry points
models/demo_7dof/         # Public educational URDF (primitives, MIT-safe)
assets/proprietary/       # Local-only company/robot CAD (gitignored)
docs/                     # Pipeline & learning notes
results/examples/         # Example output figures
```

| Module | Role |
|--------|------|
| `param_id/regressor.py` | \(Y_g\) via RNEA; `computeJointTorqueRegressor` + friction |
| `param_id/base_params.py` | Column-pivoted QR → base parameters |
| `param_id/estimators.py` | OLS / Huber-IRLS / whitened robust WLS |
| `param_id/trajectory.py` | Fourier & cosine excitation |
| `scripts/collision_view.py` | Trajectory replay + collision check (Pinocchio/hppfcl + Trimesh) |

---

## Example results (simulation)

| Experiment | What to look at |
|------------|-----------------|
| Statics + robust WLS | Gravity compensation error ≪ all-sample RMSE (outliers inflate the latter) |
| Dynamics + robust WLS | QR rank (e.g. ~59/84 on a 7-DoF left-arm model); inlier torque RMSE |
| Collision view | `COLLISION: green` on Fourier / cosine / CV trajectories |

Example figures: [`results/examples/`](results/examples/).

---

## Method references

- Statics / base-parameter ideas: gravity + friction identification literature (e.g. [ScienceDirect 2019](https://www.sciencedirect.com/science/article/pii/S0736584518304411))  
- Dynamics identification / robust regression: e.g. [IEEE Xplore 9097291](https://ieeexplore.ieee.org/abstract/document/9097291/)  
- Implementation stack: [Pinocchio](https://stack-of-tasks.github.io/pinocchio/)

---

## Disclaimer / IP

- **Do not** publish proprietary robot URDF/meshes from an employer without written permission.  
- This repo’s default model is an **educational primitive arm** under MIT.  
- Optional SDP physical-feasibility constraints (cvxpy) are not enabled by default.

---

## Author

**Tong Xiaoke (仝小可)** — Control Science & Engineering, Northeastern University  
GitHub: [txk1228](https://github.com/txk1228)

---

## License

MIT — see [LICENSE](LICENSE).
