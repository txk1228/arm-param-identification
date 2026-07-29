#!/usr/bin/env bash
# 一键演示（教学 7-DoF）— 中文终端输出。
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
export PARAM_ID_URDF="${PARAM_ID_URDF:-$ROOT/models/demo_7dof/demo_arm.urdf}"
export PARAM_ID_LANG=zh

echo "[演示] URDF=$PARAM_ID_URDF"
echo "[演示] ① 自检 → ② 静力学辨识 → ③ 动力学辨识 → ④ 碰撞检查"
python scripts/00_sanity_check.py
python scripts/identify_static.py  --method robust_wls --duration 15 --outlier-ratio 0.05
python scripts/identify_dynamic.py --method robust_wls --n-periods 2 --subsample 4 --outlier-ratio 0.05
python scripts/collision_view.py   --traj fourier --check-only --n-periods 1
echo "[演示] 完成。读数提示：优先看「力矩 RMSE（内点）」与「重力补偿误差」；图在 results/"
