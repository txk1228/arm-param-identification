#!/usr/bin/env python3
"""Static parameter identification: gravity + Coulomb friction.

Pipeline (matches internship PDF):
  1) Numeric Y_g via RNEA finite differences
  2) Pivoted QR -> base columns
  3) Smooth Coulomb Y_f
  4) OLS / Huber-IRLS / robust WLS

Usage:
  conda activate env_isaaclab
  cd ~/txk/param_id
  python scripts/identify_static.py
  python scripts/identify_static.py --method robust_wls --outlier-ratio 0.05
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))

from param_id.base_params import select_base_columns
from param_id.estimators import irls_huber, ols, robust_wls
from param_id.regressor import gravity_params_from_model, static_regressor
from param_id.robot_model import build_model, joint_limits
from param_id.trajectory import cosine_static_trajectory


def synthesize_static_data(
    model: pin.Model,
    data: pin.Data,
    q: np.ndarray,
    dq: np.ndarray,
    pi_g: np.ndarray,
    pi_fc: np.ndarray,
    noise_std: float,
    outlier_ratio: float,
    outlier_scale: float,
    hetero_noise_scale: float,
    rng: np.random.Generator,
    K_coulomb: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return stacked Y, tau, and outlier mask (per sample)."""
    nv = model.nv
    n = q.shape[0]
    Y_rows = []
    tau_rows = []
    for i in range(n):
        Yi = static_regressor(model, data, q[i], dq[i], K_coulomb=K_coulomb)
        pi = np.concatenate([pi_g, pi_fc])
        tau_i = Yi @ pi
        Y_rows.append(Yi)
        tau_rows.append(tau_i)
    Y = np.vstack(Y_rows)
    tau = np.concatenate(tau_rows)

    # Heteroscedastic noise: distal joints noisier
    scales = np.linspace(1.0, hetero_noise_scale, nv)
    noise = rng.normal(0.0, 1.0, size=(n, nv)) * (noise_std * scales)[None, :]
    tau = tau + noise.reshape(-1)

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
    p = argparse.ArgumentParser(description="Static parameter identification demo")
    p.add_argument("--method", choices=["ols", "huber", "robust_wls"], default="huber")
    p.add_argument("--fs", type=float, default=50.0)
    p.add_argument("--duration", type=float, default=30.0)
    p.add_argument("--noise-std", type=float, default=0.05)
    p.add_argument("--outlier-ratio", type=float, default=0.05)
    p.add_argument("--outlier-scale", type=float, default=50.0)
    p.add_argument("--hetero-noise-scale", type=float, default=3.0)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--urdf", type=str, default=None, help="URDF path (default: auto)")
    args = p.parse_args()

    rng = np.random.default_rng(args.seed)
    model, data, names, urdf = build_model(args.urdf)
    q_min, q_max = joint_limits(model)
    nv = model.nv
    print(f"[static] urdf={urdf.name}")
    print(f"[static] model nv={nv}, joints={names}")

    t, q, dq = cosine_static_trajectory(
        q_min, q_max, fs=args.fs, duration=args.duration
    )
    print(f"[static] samples={len(t)}, duration={args.duration}s")

    pi_g_true = gravity_params_from_model(model)
    pi_fc_true = rng.uniform(1.0, 5.0, size=nv)
    print(f"[static] gravity params={pi_g_true.size}, friction={nv}")

    Y, tau, outlier_mask = synthesize_static_data(
        model,
        data,
        q,
        dq,
        pi_g_true,
        pi_fc_true,
        noise_std=args.noise_std,
        outlier_ratio=args.outlier_ratio,
        outlier_scale=args.outlier_scale,
        hetero_noise_scale=args.hetero_noise_scale,
        rng=rng,
        K_coulomb=300.0,
    )

    # QR on gravity part only then append friction (always identifiable)
    n_g = pi_g_true.size
    Yg = Y[:, :n_g]
    Yf = Y[:, n_g:]
    # Stack a subset for QR rank (use all)
    idx_g, _, diag = select_base_columns(Yg)
    print(f"[static] QR: full gravity cols={n_g}, base={len(idx_g)}")
    print(f"         R diag head={np.array2string(diag[:8], precision=3)}")

    Yb = np.hstack([Y[:, idx_g], Yf])
    pi_true_b = np.concatenate([pi_g_true[idx_g], pi_fc_true])

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

    print(f"[static] method={args.method}")
    print(f"         torque RMSE (all)    = {rmse:.4f} N·m")
    print(f"         torque RMSE (inlier) = {rmse_in:.4f} N·m")
    print(f"         base-param relative error = {rel_param:.4f}")
    if args.method == "robust_wls":
        print(f"         rejected samples = {info.get('n_rejected')} / {len(t)}")
        print(f"         true outliers     = {int(outlier_mask.sum())}")

    # Predict gravity compensation at a few postures (zero velocity friction~0)
    q_test = q[:: max(1, len(q) // 5)]
    print("[static] gravity compensation check (||tau_g_true - tau_g_pred||):")
    for i, qi in enumerate(q_test[:5]):
        Ygi = static_regressor(model, data, qi, np.zeros(nv))[:, :n_g][:, idx_g]
        # true gravity only
        tau_g = pin.rnea(model, data, qi, np.zeros(nv), np.zeros(nv))
        tau_g_hat = Ygi @ pi_hat[: len(idx_g)]
        print(f"  posture {i}: err={np.linalg.norm(tau_g - tau_g_hat):.4f} N·m")

    # Plot
    out_dir = Path(__file__).resolve().parents[1] / "results"
    out_dir.mkdir(exist_ok=True)
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    axes[0].plot(t, tau.reshape(-1, nv)[:, 0], lw=0.8, label="meas j0")
    axes[0].plot(t, tau_hat.reshape(-1, nv)[:, 0], lw=0.8, label="pred j0")
    if outlier_mask.any():
        axes[0].scatter(
            t[outlier_mask],
            tau.reshape(-1, nv)[outlier_mask, 0],
            c="C3",
            s=12,
            zorder=3,
            label="outlier sample",
        )
    axes[0].set_ylabel("tau [N·m]")
    axes[0].legend(loc="upper right")
    axes[0].set_title(f"Static ID ({args.method}) — joint 0 torque")
    axes[1].plot(t, resid[:, 0], lw=0.8)
    axes[1].set_xlabel("t [s]")
    axes[1].set_ylabel("residual j0")
    fig.tight_layout()
    fig_path = out_dir / f"static_{args.method}.png"
    fig.savefig(fig_path, dpi=140)
    print(f"[static] saved figure -> {fig_path}")

    np.savez(
        out_dir / f"static_{args.method}.npz",
        pi_hat=pi_hat,
        pi_true_b=pi_true_b,
        idx_g=idx_g,
        rmse=rmse,
        rel_param=rel_param,
        joint_names=np.array(names),
    )


if __name__ == "__main__":
    main()
