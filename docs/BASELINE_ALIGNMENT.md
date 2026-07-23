# Baseline alignment (ideal physics)

Ideal conditions: gravity (+ inertia for dynamics) only; joint Coulomb/viscous friction and armature off on Isaac (`--ideal-physics`). Same excitation seed/params on both sides. Identification uses OLS.

**中文版：** [`zh/BASELINE_ALIGNMENT.md`](zh/BASELINE_ALIGNMENT.md)

## Pass gate

| Experiment | Metric | Limit |
|------------|--------|------:|
| Statics (cosine) | Gravity **base** param relative error vs URDF QR projection | **< 5%** |
| Dynamics (fourier) | Full base-param relative error vs URDF QR projection | **< 5%** |

Also report torque-fit RMSE and cross difference `|π̂^P - π̂^I|`.

## A) Statics — cosine

```bash
conda activate env_isaaclab
cd /path/to/param_id
export PARAM_ID_URDF=$PWD/models/demo_7dof/demo_arm.urdf

# 1) Pinocchio ideal dataset (RNEA, no friction)
python scripts/export_pinocchio_dataset.py --mode static --traj cosine \
  --n-periods 2 --dt 0.01 --seed 0 --fundamental-freq 0.1 \
  --save-path results/baseline/pinocchio_static_cosine.npz

# 2) Isaac ideal collection
python scripts/collect_data_isaaclab.py --mode static --traj cosine \
  --n-periods 2 --dt 0.01 --seed 0 --fundamental-freq 0.1 \
  --ddq-mode ideal --ideal-physics --headless \
  --save-path results/baseline/isaac_static_cosine_ideal.npz

# 3) Identify (same method)
python scripts/identify_static.py --method ols --data-source file \
  --data-path results/baseline/pinocchio_static_cosine.npz \
  --results-dir results/baseline/id_pinocchio_static
python scripts/identify_static.py --method ols --data-source file \
  --data-path results/baseline/isaac_static_cosine_ideal.npz \
  --results-dir results/baseline/id_isaac_static

# 4) Compare + bar charts + PASS/FAIL
python scripts/compare_baseline_alignment.py --kind static \
  --pinocchio-data results/baseline/pinocchio_static_cosine.npz \
  --isaac-data results/baseline/isaac_static_cosine_ideal.npz \
  --out-dir results/baseline/compare_static
```

## B) Dynamics — fourier

```bash
python scripts/export_pinocchio_dataset.py --mode dynamic --traj fourier \
  --n-periods 2 --fourier-harmonics 5 --dt 0.01 --seed 0 \
  --save-path results/baseline/pinocchio_dynamic_fourier.npz

python scripts/collect_data_isaaclab.py --mode dynamic --traj fourier \
  --n-periods 2 --fourier-harmonics 5 --dt 0.01 --seed 0 \
  --ddq-mode ideal --ideal-physics --headless \
  --save-path results/baseline/isaac_dynamic_fourier_ideal.npz

python scripts/identify_dynamic.py --method ols --data-source file --subsample 1 \
  --data-path results/baseline/pinocchio_dynamic_fourier.npz \
  --results-dir results/baseline/id_pinocchio_dynamic
python scripts/identify_dynamic.py --method ols --data-source file --subsample 1 \
  --data-path results/baseline/isaac_dynamic_fourier_ideal.npz \
  --results-dir results/baseline/id_isaac_dynamic

python scripts/compare_baseline_alignment.py --kind dynamic \
  --pinocchio-data results/baseline/pinocchio_dynamic_fourier.npz \
  --isaac-data results/baseline/isaac_dynamic_fourier_ideal.npz \
  --out-dir results/baseline/compare_dynamic
```

One-shot: `bash scripts/run_baseline_alignment.sh`
