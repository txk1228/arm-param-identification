#!/usr/bin/env bash
# Baseline alignment (ideal physics): Pinocchio vs Isaac Lab.
# Usage:
#   conda activate env_isaaclab
#   bash scripts/run_baseline_alignment.sh
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
export PARAM_ID_URDF="${PARAM_ID_URDF:-$ROOT/models/demo_7dof/demo_arm.urdf}"
PY="${PYTHON:-python}"
mkdir -p results/baseline

N_PERIODS=2
DT=0.01
SEED=0
HARM=5
FFREQ=0.1

echo "========== A) STATICS: cosine =========="
$PY scripts/export_pinocchio_dataset.py --mode static --traj cosine \
  --n-periods "$N_PERIODS" --dt "$DT" --seed "$SEED" --fundamental-freq "$FFREQ" \
  --urdf "$PARAM_ID_URDF" \
  --save-path results/baseline/pinocchio_static_cosine.npz

$PY scripts/collect_data_isaaclab.py --mode static --traj cosine \
  --n-periods "$N_PERIODS" --dt "$DT" --seed "$SEED" --fundamental-freq "$FFREQ" \
  --ddq-mode ideal --ideal-physics --headless --warmup-steps 50 \
  --urdf "$PARAM_ID_URDF" \
  --save-path results/baseline/isaac_static_cosine_ideal.npz

$PY scripts/identify_static.py --method ols --data-source file \
  --data-path results/baseline/pinocchio_static_cosine.npz --urdf "$PARAM_ID_URDF" \
  --results-dir results/baseline/id_pinocchio_static

$PY scripts/identify_static.py --method ols --data-source file \
  --data-path results/baseline/isaac_static_cosine_ideal.npz --urdf "$PARAM_ID_URDF" \
  --results-dir results/baseline/id_isaac_static

$PY scripts/compare_baseline_alignment.py --kind static \
  --pinocchio-data results/baseline/pinocchio_static_cosine.npz \
  --isaac-data results/baseline/isaac_static_cosine_ideal.npz \
  --urdf "$PARAM_ID_URDF" \
  --out-dir results/baseline/compare_static

echo "========== B) DYNAMICS: fourier =========="
$PY scripts/export_pinocchio_dataset.py --mode dynamic --traj fourier \
  --n-periods "$N_PERIODS" --fourier-harmonics "$HARM" --dt "$DT" --seed "$SEED" \
  --fundamental-freq "$FFREQ" --urdf "$PARAM_ID_URDF" \
  --save-path results/baseline/pinocchio_dynamic_fourier.npz

$PY scripts/collect_data_isaaclab.py --mode dynamic --traj fourier \
  --n-periods "$N_PERIODS" --fourier-harmonics "$HARM" --dt "$DT" --seed "$SEED" \
  --fundamental-freq "$FFREQ" \
  --ddq-mode ideal --ideal-physics --headless --warmup-steps 50 \
  --urdf "$PARAM_ID_URDF" \
  --save-path results/baseline/isaac_dynamic_fourier_ideal.npz

$PY scripts/identify_dynamic.py --method ols --data-source file --subsample 1 \
  --data-path results/baseline/pinocchio_dynamic_fourier.npz --urdf "$PARAM_ID_URDF" \
  --results-dir results/baseline/id_pinocchio_dynamic

$PY scripts/identify_dynamic.py --method ols --data-source file --subsample 1 \
  --data-path results/baseline/isaac_dynamic_fourier_ideal.npz --urdf "$PARAM_ID_URDF" \
  --results-dir results/baseline/id_isaac_dynamic

$PY scripts/compare_baseline_alignment.py --kind dynamic \
  --pinocchio-data results/baseline/pinocchio_dynamic_fourier.npz \
  --isaac-data results/baseline/isaac_dynamic_fourier_ideal.npz \
  --urdf "$PARAM_ID_URDF" \
  --out-dir results/baseline/compare_dynamic

echo "Done. See results/baseline/compare_*/"
