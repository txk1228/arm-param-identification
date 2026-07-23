#!/usr/bin/env python3
"""Dynamic parameter identification: inertia + Coriolis + gravity + friction.

Pipeline:
  1) Pinocchio computeJointTorqueRegressor + Coulomb/viscous columns
  2) Pivoted QR base parameters
  3) Column-normalized WLS + whitening + Huber (+ optional outer hard reject)
  4) Fourier excitation, noise & outliers

Usage:
  python scripts/identify_dynamic.py
  python scripts/identify_dynamic.py --method robust_wls --outlier-ratio 0.05
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))

from param_id.base_params import select_base_columns
from param_id.estimators import irls_huber, ols, robust_wls
from param_id.regressor import dynamics_regressor
from param_id.robot_model import build_model, extract_inertial_params, joint_limits
from param_id.trajectory import fourier_trajectory


def synthesize_dynamic_data(
    model,
    data,
    q,
    dq,
    ddq,
    pi_true,
    noise_std: float,
    outlier_ratio: float,
    outlier_scale: float,
    rng: np.random.Generator,
    K_coulomb: float,
):
    nv = model.nv
    n = q.shape[0]
    Y_rows = []
    tau_rows = []
    for i in range(n):
        Yi = dynamics_regressor(
            model, data, q[i], dq[i], ddq[i], K_coulomb=K_coulomb
        )
        Y_rows.append(Yi)
        tau_rows.append(Yi @ pi_true)
    Y = np.vstack(Y_rows)
    tau = np.concatenate(tau_rows)
    tau = tau + rng.normal(0.0, noise_std, size=tau.shape)

    outlier_mask = np.zeros(n, dtype=bool)
    n_out = int(n * outlier_ratio)
    if n_out > 0:
        idx = rng.choice(n, size=n_out, replace=False)
        outlier_mask[idx] = True
        for i in idx:
            j = int(rng.integers(0, nv))
            tau[i * nv + j] += rng.choice([-1.0, 1.0]) * outlier_scale * rng.uniform(
                0.5, 1.5
            )
    return Y, tau, outlier_mask


def main() -> None:
    p = argparse.ArgumentParser(description="Dynamic parameter identification demo")
    p.add_argument("--method", choices=["ols", "huber", "robust_wls"], default="robust_wls")
    p.add_argument("--fs", type=float, default=100.0)
    p.add_argument("--fundamental-freq", type=float, default=0.1)
    p.add_argument("--n-periods", type=int, default=5)
    p.add_argument("--harmonics", type=int, default=5)
    p.add_argument("--noise-std", type=float, default=0.05)
    p.add_argument("--outlier-ratio", type=float, default=0.05)
    p.add_argument("--outlier-scale", type=float, default=50.0)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--subsample", type=int, default=2, help="Use every k-th sample")
    p.add_argument("--urdf", type=str, default=None, help="URDF path (default: auto)")
    args = p.parse_args()

    rng = np.random.default_rng(args.seed)
    model, data, names, urdf = build_model(args.urdf)
    q_min, q_max = joint_limits(model)
    nv = model.nv
    print(f"[dynamic] urdf={urdf.name}")
    print(f"[dynamic] model nv={nv}, joints={names}")

    t, q, dq, ddq = fourier_trajectory(
        q_min,
        q_max,
        fs=args.fs,
        fundamental_freq=args.fundamental_freq,
        n_periods=args.n_periods,
        harmonics=args.harmonics,
        rng=rng,
    )
    # Subsample to keep QR/WLS manageable for learning runs
    sl = slice(None, None, args.subsample)
    t, q, dq, ddq = t[sl], q[sl], dq[sl], ddq[sl]
    print(f"[dynamic] samples={len(t)} (subsample={args.subsample})")

    pi_inertial = extract_inertial_params(model)
    pi_fc = rng.uniform(1.0, 5.0, size=nv)
    pi_fv = rng.uniform(0.1, 0.5, size=nv)
    pi_true = np.concatenate([pi_inertial, pi_fc, pi_fv])
    print(
        f"[dynamic] params: inertial={pi_inertial.size}, fc={nv}, fv={nv}, total={pi_true.size}"
    )

    Y, tau, outlier_mask = synthesize_dynamic_data(
        model,
        data,
        q,
        dq,
        ddq,
        pi_true,
        noise_std=args.noise_std,
        outlier_ratio=args.outlier_ratio,
        outlier_scale=args.outlier_scale,
        rng=rng,
        K_coulomb=300.0,
    )

    idx, Yb, diag = select_base_columns(Y)
    print(f"[dynamic] QR: full cols={Y.shape[1]}, base={len(idx)}")
    print(f"          R diag head={np.array2string(diag[:10], precision=2)}")
    pi_true_b = pi_true[idx]

    if args.method == "ols":
        pi_hat = ols(Yb, tau)
        info = {}
    elif args.method == "huber":
        pi_hat, w = irls_huber(Yb, tau)
        info = {"huber_mean_w": float(w.mean())}
    else:
        pi_hat, info = robust_wls(Yb, tau, nv=nv)

    tau_hat = Yb @ pi_hat
    resid = (tau - tau_hat).reshape(-1, nv)
    rmse = float(np.sqrt(np.mean(resid**2)))
    inlier = ~outlier_mask
    rmse_in = float(np.sqrt(np.mean(resid[inlier] ** 2))) if inlier.any() else rmse
    rel_param = float(
        np.linalg.norm(pi_hat - pi_true_b) / (np.linalg.norm(pi_true_b) + 1e-12)
    )

    print(f"[dynamic] method={args.method}")
    print(f"          torque RMSE (all)    = {rmse:.4f} N·m")
    print(f"          torque RMSE (inlier) = {rmse_in:.4f} N·m")
    print(f"          base-param relative error = {rel_param:.4f}")
    if args.method == "robust_wls":
        print(f"          rejected samples = {info.get('n_rejected')} / {len(t)}")
        print(f"          true outliers     = {int(outlier_mask.sum())}")

    # Cross-check: hold-out last 20% time for prediction RMSE
    n = len(t)
    n_te = max(1, n // 5)
    # rebuild Y_te quickly from stored arrays
    Y_te = Y[-n_te * nv :]
    tau_te = tau[-n_te * nv :]
    Yb_te = Y_te[:, idx]
    rmse_te = float(np.sqrt(np.mean((tau_te - Yb_te @ pi_hat) ** 2)))
    print(f"          hold-out torque RMSE = {rmse_te:.4f} N·m")

    out_dir = Path(__file__).resolve().parents[1] / "results"
    out_dir.mkdir(exist_ok=True)
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=False)
    axes[0].plot(t, q[:, 0], label="q0")
    axes[0].plot(t, q[:, 3], label="q3")
    axes[0].set_ylabel("q [rad]")
    axes[0].legend()
    axes[0].set_title("Fourier excitation (sample joints)")
    axes[1].plot(t, tau.reshape(-1, nv)[:, 0], lw=0.7, label="meas")
    axes[1].plot(t, tau_hat.reshape(-1, nv)[:, 0], lw=0.7, label="pred")
    if outlier_mask.any():
        axes[1].scatter(
            t[outlier_mask],
            tau.reshape(-1, nv)[outlier_mask, 0],
            c="C3",
            s=10,
            label="outlier",
        )
    axes[1].legend()
    axes[1].set_ylabel("tau0")
    axes[2].bar(np.arange(len(pi_true_b)), pi_true_b, alpha=0.5, label="true base")
    axes[2].bar(np.arange(len(pi_hat)), pi_hat, alpha=0.5, label="est base")
    axes[2].set_xlabel("base param index")
    axes[2].legend()
    fig.tight_layout()
    fig_path = out_dir / f"dynamic_{args.method}.png"
    fig.savefig(fig_path, dpi=140)
    print(f"[dynamic] saved figure -> {fig_path}")

    np.savez(
        out_dir / f"dynamic_{args.method}.npz",
        pi_hat=pi_hat,
        pi_true_b=pi_true_b,
        idx=idx,
        rmse=rmse,
        rmse_te=rmse_te,
        rel_param=rel_param,
        joint_names=np.array(names),
    )


if __name__ == "__main__":
    main()
