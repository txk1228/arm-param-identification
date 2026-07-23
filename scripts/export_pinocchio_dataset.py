#!/usr/bin/env python3
"""Export Pinocchio ideal-physics datasets matching Isaac collection params.

Ideal conditions: gravity (+ inertia for dynamics) only — **no** Coulomb/viscous
friction, no measurement noise. Torque from RNEA on the shared excitation.

Usage::

    python scripts/export_pinocchio_dataset.py --mode static --traj cosine \\
        --n-periods 2 --dt 0.01 --seed 0 \\
        --save-path results/baseline/pinocchio_static_cosine.npz

    python scripts/export_pinocchio_dataset.py --mode dynamic --traj fourier \\
        --n-periods 2 --fourier-harmonics 5 --dt 0.01 --seed 0 \\
        --save-path results/baseline/pinocchio_dynamic_fourier.npz
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))

from param_id.robot_model import build_model, joint_limits
from utils.data_io import save_dataset
from utils.traj_generator import generate_excitation


def main() -> None:
    p = argparse.ArgumentParser(description="Export ideal Pinocchio NPZ dataset")
    p.add_argument("--mode", choices=["static", "dynamic"], required=True)
    p.add_argument("--traj", choices=["fourier", "cosine"], required=True)
    p.add_argument("--n-periods", type=int, default=2)
    p.add_argument("--fourier-harmonics", type=int, default=5)
    p.add_argument("--fundamental-freq", type=float, default=0.1)
    p.add_argument("--dt", type=float, default=0.01)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--urdf", type=str, default=None)
    p.add_argument("--save-path", type=str, required=True)
    args = p.parse_args()

    model, data, names, urdf = build_model(args.urdf)
    q_min, q_max = joint_limits(model)
    nv = model.nv

    t, q, dq, ddq = generate_excitation(
        args.traj,
        q_min,
        q_max,
        dt=args.dt,
        n_periods=args.n_periods,
        fourier_harmonics=args.fourier_harmonics,
        fundamental_freq=args.fundamental_freq,
        seed=args.seed,
    )

    tau = np.zeros_like(q)
    zero = np.zeros(nv)
    if args.mode == "static":
        # Ideal statics: gravity only (dq/ddq ignored by RNEA here).
        for i in range(q.shape[0]):
            tau[i] = pin.rnea(model, data, q[i], zero, zero)
        # Keep analytic dq from traj for friction columns (~0 effect if small).
    else:
        for i in range(q.shape[0]):
            tau[i] = pin.rnea(model, data, q[i], dq[i], ddq[i])

    traj_type = f"pinocchio_{args.mode}_{args.traj}_ideal"
    path = save_dataset(
        args.save_path,
        {
            "q": q,
            "dq": dq if args.mode == "dynamic" else np.zeros_like(dq),
            "ddq": ddq if args.mode == "dynamic" else np.zeros_like(ddq),
            "tau": tau,
            "dt": float(args.dt),
            "traj_type": traj_type,
        },
    )
    print(f"[pinocchio-export] urdf={urdf.name} joints={names}")
    print(f"[pinocchio-export] mode={args.mode} traj={args.traj} N={len(t)}")
    print(f"[pinocchio-export] tau RMS={np.sqrt(np.mean(tau**2)):.4f} Nm")
    print(f"[pinocchio-export] saved -> {path}")


if __name__ == "__main__":
    main()
