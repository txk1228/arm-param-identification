#!/usr/bin/env bash
# One-shot public demo (educational 7-DoF arm).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
export PARAM_ID_URDF="${PARAM_ID_URDF:-$ROOT/models/demo_7dof/demo_arm.urdf}"

echo "[demo] URDF=$PARAM_ID_URDF"
python scripts/00_sanity_check.py
python scripts/identify_static.py  --method robust_wls --duration 15 --outlier-ratio 0.05
python scripts/identify_dynamic.py --method robust_wls --n-periods 2 --subsample 4 --outlier-ratio 0.05
python scripts/collision_view.py   --traj fourier --check-only --n-periods 1
echo "[demo] done. See results/"
