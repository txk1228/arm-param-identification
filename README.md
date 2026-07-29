# 7-DoF Arm Dynamics / Statics Parameter Identification

[![Python 3.11](https://img.shields.io/badge/python-3.11-blue.svg)](https://www.python.org/)
[![Pinocchio](https://img.shields.io/badge/Pinocchio-2.x-green.svg)](https://stack-of-tasks.github.io/pinocchio/)
[![Isaac Lab](https://img.shields.io/badge/Isaac_Lab-optional-orange.svg)](https://isaac-sim.github.io/IsaacLab/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Simulation and identification pipeline for **robot inertial and friction parameters** on a 7-DoF manipulator. The **algorithm layer** (regressors, QR base parameters, OLS / Huber / robust WLS) is decoupled from the **data layer** (Pinocchio synthesis or Isaac Lab collection) via a unified NPZ interface.

| Layer | Path | Role |
|-------|------|------|
| Algorithm | `param_id/` | Regressors, pivoted QR, OLS / Huber / whitened robust WLS |
| Data I/O | `utils/` | NPZ schema, trajectories, PD law, noise, gravity feedforward |
| Collection | `scripts/collect_data_isaaclab.py` | Optional PhysX acquisition (position servo / PD torque, friction, noise) |
| Validation | `scripts/compare_*`, `verify_gravity_compensation.py` | Baseline alignment, cross-comparison, closed-loop GC |

> **Scope:** Default Pinocchio paths synthesize torques from URDF ground truth (+ noise/outliers). Reported errors evaluate **methods and pipelines**, not hardware identification accuracy.

**中文说明：** [`docs/zh/README.md`](docs/zh/README.md)

---

## Documentation

| Doc | Description |
|-----|-------------|
| [`docs/README.md`](docs/README.md) | Full documentation index (EN + 中文) |
| [`docs/zh/METHOD.md`](docs/zh/METHOD.md) | **算法原理（中文）** / [EN](docs/METHOD.md) |
| [`docs/zh/LEARNING.md`](docs/zh/LEARNING.md) | **管线复现（中文）** / [EN](docs/LEARNING.md) |
| [`docs/zh/BASELINE_ALIGNMENT.md`](docs/zh/BASELINE_ALIGNMENT.md) | Baseline alignment（中文） |
| [`docs/zh/RESULTS_ANALYSIS.md`](docs/zh/RESULTS_ANALYSIS.md) | How to read comparison figures |
| [`docs/zh/README.md`](docs/zh/README.md) | 中文总览 |
| [`results/README.md`](results/README.md) | Output directory layout |

---

## Pipeline

```text
URDF (demo 7-DoF or your arm)
        │
        ├─ Pinocchio synthesize  ──┐
        │   (RNEA + optional noise) │
        │                           │
        └─ Isaac Lab collect ───────┼──► unified NPZ {q, dq, ddq, tau, dt, traj_type}
                                    │
                                    ▼
                     identify_static / identify_dynamic
                     (--data-source pinocchio | file)
                                    │
                                    ▼
                       pivoted QR → OLS / Huber / robust WLS
                                    │
                     ┌──────────────┼──────────────┐
                     ▼              ▼              ▼
               plots / NPZ   results/comparison/   gravity compensation
```

---

## Repository structure

```text
param_id/                          # Project root
│
├── param_id/                      # Core identification package
│   ├── regressor.py               # Gravity / dynamics / friction regressors
│   ├── base_params.py             # Column-pivoted QR base-parameter selection
│   ├── estimators.py              # OLS, Huber-IRLS, whitened robust WLS
│   ├── trajectory.py              # Fourier & cosine excitation
│   └── robot_model.py             # URDF resolve, Pinocchio model build
│
├── utils/                         # Data layer (decoupled from algorithms)
│   ├── data_io.py                 # NPZ save/load + validation
│   ├── traj_generator.py          # Trajectory facade
│   ├── ddq.py                     # Ideal vs measured acceleration
│   ├── collect_extras.py          # PD law, Gaussian noise, outliers
│   └── gravity_comp.py            # Gravity torque from URDF or ID result
│
├── scripts/                       # CLI entry points
│   ├── 00_sanity_check.py         # Regressor vs RNEA sanity check
│   ├── identify_static.py         # Static ID (gravity + Coulomb friction)
│   ├── identify_dynamic.py        # Dynamic ID (inertia + friction)
│   ├── collect_data_isaaclab.py   # Isaac Lab data collection
│   ├── export_pinocchio_dataset.py# Export ideal Pinocchio NPZ
│   ├── compare_baseline_alignment.py
│   ├── run_comparison.py          # Three-arm cross-comparison
│   ├── verify_gravity_compensation.py
│   ├── collision_view.py          # Trajectory collision check / viewer
│   ├── run_demo.sh                # One-shot Pinocchio demo
│   ├── run_baseline_alignment.sh
│   └── run_comparison_experiments.sh
│
├── models/demo_7dof/              # Public educational URDF (MIT)
│   └── demo_arm.urdf
│
├── configs/
│   └── comparison_experiment.yaml # Stage-6 experiment parameters
│
├── assets/proprietary/            # Local private CAD (gitignored)
│   └── README.md
│
├── docs/                          # Documentation (see docs/README.md)
│   ├── METHOD.md, LEARNING.md, BASELINE_ALIGNMENT.md, UPLOAD.md
│   └── zh/                        # Chinese docs
│       ├── README.md
│       └── RESULTS_ANALYSIS.md
│
├── results/                       # Local outputs (see results/README.md)
│   ├── examples/                  # Committed example figures
│   ├── comparison/                # Cross-comparison showcase
│   ├── gravity_comp/              # Gravity-comp verification
│   └── baseline/                  # Alignment NPZ & metrics
│
├── environment.yml                # Conda env (Pinocchio path)
├── requirements.txt               # Pip deps (Isaac env supplement)
├── pyrightconfig.json
└── LICENSE
```

`meshes/` and `assets/proprietary/urdf/` are gitignored — do not commit employer CAD.

---

## Environment

```bash
cd /path/to/param_id

# Option A: lightweight conda (Pinocchio only)
conda env create -f environment.yml
conda activate param-id

# Option B: Isaac Lab env (PhysX collection + validation)
conda activate env_isaaclab
pip install -r requirements.txt   # if anything is missing
```

Set the demo URDF (or your own):

```bash
export PARAM_ID_URDF=$PWD/models/demo_7dof/demo_arm.urdf
```

**Cursor / VS Code:** select the conda env `env_isaaclab` (or `param-id`) interpreter — not system `/bin/python3` (missing `matplotlib` / `pinocchio`).

---

## Quick start (Pinocchio only)

No Isaac required.

```bash
bash scripts/run_demo.sh
```

Or step by step:

```bash
python scripts/00_sanity_check.py
python scripts/identify_static.py  --method robust_wls --outlier-ratio 0.05
python scripts/identify_dynamic.py --method robust_wls --outlier-ratio 0.05 --n-periods 3
python scripts/collision_view.py   --traj fourier --check-only
```

Outputs land in `results/` (NPZ + PNG). Custom URDF: `--urdf /path/to/arm.urdf`.

---

## Isaac Lab demo (GUI + headless)

There is **no separate official Isaac scene pack**. The project demo is  
`scripts/collect_data_isaaclab.py` (load URDF → track excitation → write NPZ).

### Prerequisites

```bash
conda activate env_isaaclab
cd /path/to/param_id
export PARAM_ID_URDF=$PWD/models/demo_7dof/demo_arm.urdf
```

> **Import order:** scripts import Pinocchio **before** `AppLauncher`.  
> Starting Kit first then loading Pinocchio can make the app exit right after  
> `Simulation App Startup Complete` (pybind conflict). Do not reorder those imports.

### A) GUI demo — visualize tracking

Omit `--headless` so Isaac Sim opens a window:

```bash
python scripts/collect_data_isaaclab.py \
  --mode dynamic --traj fourier --n-periods 1 --dt 0.01 \
  --ddq-mode ideal --control-mode position_servo \
  --save-path results/baseline/isaac_demo_fourier.npz
```

Optional: `--device cuda:0` (AppLauncher). Close the window or wait until recording finishes; NPZ is written under `results/baseline/`.

### B) Headless demo — data only (no window)

```bash
python scripts/collect_data_isaaclab.py \
  --mode dynamic --traj fourier --n-periods 1 --dt 0.01 \
  --ddq-mode ideal --headless \
  --save-path results/baseline/isaac_demo_fourier.npz
```

### C) Engineering collection (closer to hardware)

```bash
python scripts/collect_data_isaaclab.py \
  --mode dynamic --traj fourier --n-periods 2 --dt 0.01 --seed 0 \
  --control-mode pd_torque --kp 200 --kd 20 \
  --enable-friction --q-noise-std 1e-4 --tau-noise-std 0.05 \
  --ddq-mode ideal --headless \
  --save-path results/baseline/isaac_eng_fourier.npz
```

For a GUI engineering run, drop `--headless` from the same command.

### D) Identify from the collected NPZ

```bash
python scripts/identify_dynamic.py --method robust_wls \
  --data-source file --data-path results/baseline/isaac_demo_fourier.npz \
  --results-dir results/isaac_dynamic
```

### E) Gravity-compensation demo (Isaac)

```bash
# Headless hold + drag metrics
python scripts/verify_gravity_compensation.py --headless \
  --id-result results/static_ols.npz --out-dir results/gravity_comp

# GUI: omit --headless
python scripts/verify_gravity_compensation.py \
  --id-result results/static_ols.npz --out-dir results/gravity_comp
```

Offline (no Isaac): add `--offline`.

---

## Unified dataset format

All datasets use compressed NPZ via `utils/data_io.py`:

| Field | Type | Shape / value |
|-------|------|----------------|
| `q` | `float64` | `(N, n_joint)` |
| `dq` | `float64` | `(N, n_joint)` |
| `ddq` | `float64` | `(N, n_joint)` |
| `tau` | `float64` | `(N, n_joint)` |
| `dt` | `float` | sample period [s] |
| `traj_type` | `str` | e.g. `isaac_dynamic_fourier_ideal` |

```python
from utils.data_io import save_dataset, load_dataset
```

---

## Workflows

Stages follow dependency order: statics → dynamics → collection → baseline alignment → cross-comparison → closed-loop verification.

### 1. Static identification (gravity compensation)

| | |
|--|--|
| Trajectory | Slow cosine |
| Estimates | Gravity-related terms + Coulomb friction |
| Use case | Gravity feedforward |

```bash
python scripts/identify_static.py --method robust_wls
```

### 2. Dynamic identification (torque feedforward)

| | |
|--|--|
| Trajectory | Fourier excitation |
| Estimates | Inertia + Coriolis/centrifugal + gravity + Coulomb/viscous |
| Use case | Model-based torque control |

```bash
python scripts/identify_dynamic.py --method robust_wls --n-periods 3
```

### 3. Isaac Lab data collection

Requires **Isaac Lab** (tested ~0.54 / Isaac Sim 5.x). Full GUI / headless demos: see **[Isaac Lab demo](#isaac-lab-demo-gui--headless)** above.

| Flag | Meaning |
|------|---------|
| `--headless` | No GUI. Omit this flag to open the Isaac Sim window |
| `--control-mode` | `position_servo` (implicit PD) or `pd_torque` (explicit τ = Kp·e + Kd·ė) |
| `--ideal-physics` | Gravity only; no joint friction (baseline alignment) |
| `--enable-friction` | PhysX static / Coulomb / viscous friction |
| `--ddq-mode ideal\|measured` | Trajectory ddq or central-diff dq + MA filter |
| `--q-noise-std` / `--tau-noise-std` | Sensor noise on logged position / torque |
| `--data-source file` | On `identify_*`: load NPZ instead of synthesizing |

### 4. Baseline alignment (ideal physics)

Same excitation on Pinocchio (RNEA, no friction) and Isaac (`--ideal-physics`). Pass gate: base-param relative error **&lt; 5%**.

```bash
bash scripts/run_baseline_alignment.sh
# Details: docs/BASELINE_ALIGNMENT.md
```

### 5. Cross-comparison showcase

Three arms → `results/comparison/`:

1. **Baseline** — Pinocchio ideal + OLS (validate pipeline)
2. **Physics** — Engineering data (PD + friction + noise) + OLS (observe degradation)
3. **Robust** — Same engineering data + robust whitened WLS (compare inlier fit)

```bash
bash scripts/run_comparison_experiments.sh          # tries Isaac; falls back to proxy
bash scripts/run_comparison_experiments.sh --skip-isaac   # offline only
```

Outputs: `fig_rmse_per_joint.png`, `fig_rel_param.png`, `fig_torque_joint0.png`, `conclusion.md`, `summary.json`.

### 6. Gravity compensation closed loop

```bash
# Offline residual metrics (no Isaac)
python scripts/verify_gravity_compensation.py --offline \
  --id-result results/static_ols.npz --out-dir results/gravity_comp

# Isaac headless: hold test + end-effector drag
python scripts/verify_gravity_compensation.py --headless \
  --id-result results/static_ols.npz --out-dir results/gravity_comp

# Isaac GUI: omit --headless
python scripts/verify_gravity_compensation.py \
  --id-result results/static_ols.npz --out-dir results/gravity_comp
```

Feedforward: `τ = Kp(q*−q) + Kd(dq*−dq) + τ_g(q)` with `τ_g` from URDF RNEA or identified `π̂_g`.

---

## Reading results

| Experiment | What to look at |
|------------|-----------------|
| Ideal baseline | Torque RMSE ≈ 0; base-param error ≈ 0% → pipeline is correct |
| Engineering OLS | High all-sample RMSE from outliers; **inlier RMSE** ~0.24 N·m |
| Engineering robust WLS | **Inlier RMSE** ~0.05 N·m (≈5× better than OLS) |
| Gravity comp offline | Identified GC residual ≈ machine precision vs RNEA |

**Important:** On engineering data, **base-parameter relative error** can exceed 200–300% because torques include friction and outliers while the reference is friction-free URDF. This does **not** mean identification failed — use **inlier torque RMSE** and gravity-compensation residual as engineering metrics. See [`docs/zh/RESULTS_ANALYSIS.md`](docs/zh/RESULTS_ANALYSIS.md).

Example figures: [`results/examples/`](results/examples/), [`results/comparison/`](results/comparison/).

---

## Method references

- Statics / base parameters: [ScienceDirect 2019](https://www.sciencedirect.com/science/article/pii/S0736584518304411)
- Dynamics ID / robust regression: [IEEE Xplore 9097291](https://ieeexplore.ieee.org/abstract/document/9097291/)
- Stack: [Pinocchio](https://stack-of-tasks.github.io/pinocchio/), [Isaac Lab](https://isaac-sim.github.io/IsaacLab/) (optional)

---

## Disclaimer

- Do **not** publish proprietary employer URDF/meshes without permission.
- Default model is an **educational primitive arm** (MIT).
- Simulation errors ≠ real-robot identification accuracy.

---

## License

MIT — see [LICENSE](LICENSE).
