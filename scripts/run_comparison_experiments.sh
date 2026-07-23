#!/usr/bin/env bash
# One-click Stage-6 cross comparison.
# Usage:
#   conda activate env_isaaclab
#   bash scripts/run_comparison_experiments.sh
#   bash scripts/run_comparison_experiments.sh --skip-isaac
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
export PARAM_ID_URDF="${PARAM_ID_URDF:-$ROOT/models/demo_7dof/demo_arm.urdf}"
PY="${PYTHON:-python}"
mkdir -p results/comparison
exec "$PY" scripts/run_comparison.py --urdf "$PARAM_ID_URDF" "$@"
