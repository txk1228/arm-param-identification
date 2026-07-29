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
  python scripts/identify_dynamic.py --data-source file --data-path results/isaac_....npz
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

from utils.cli_lang import t as tr  # noqa: E402

from param_id.base_params import select_base_columns
from param_id.estimators import irls_huber, ols, robust_wls
from param_id.regressor import dynamics_regressor
from param_id.robot_model import build_model, extract_inertial_params, joint_limits
from param_id.trajectory import fourier_trajectory
from utils.data_io import load_dataset


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


def _build_dynamic_Y(model, data, q, dq, ddq, K_coulomb: float = 300.0) -> np.ndarray:
    """Stack dynamic regressors for each sample (shared by both data sources)."""
    return np.vstack(
        [
            dynamics_regressor(model, data, q[i], dq[i], ddq[i], K_coulomb=K_coulomb)
            for i in range(q.shape[0])
        ]
    )


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
    p.add_argument(
        "--data-source",
        choices=["pinocchio", "file"],
        default="pinocchio",
        help="pinocchio: synthesize torques; file: load unified NPZ from Isaac/etc.",
    )
    p.add_argument(
        "--data-path",
        type=str,
        default=None,
        help="NPZ path when --data-source=file (q,dq,ddq,tau,...).",
    )
    p.add_argument(
        "--results-dir",
        type=str,
        default=None,
        help="Output directory (default: results/ or results/isaac_dynamic/ for file).",
    )
    args = p.parse_args()

    if args.data_source == "file" and not args.data_path:
        p.error("--data-path is required when --data-source=file")

    rng = np.random.default_rng(args.seed)
    model, data, names, urdf = build_model(args.urdf)
    q_min, q_max = joint_limits(model)
    nv = model.nv
    print(tr(f"[dynamic] urdf={urdf.name}", f"[动力学] URDF={urdf.name}"))
    print(
        tr(
            f"[dynamic] model nv={nv}, joints={names}",
            f"[动力学] 自由度 nv={nv}, 关节={names}",
        )
    )
    print(
        tr(
            f"[dynamic] data_source={args.data_source}",
            f"[动力学] 数据源={args.data_source}",
        )
    )

    pi_inertial = extract_inertial_params(model)
    K_coulomb = 300.0

    if args.data_source == "file":
        ds = load_dataset(args.data_path)
        q = np.asarray(ds["q"], dtype=float)
        dq = np.asarray(ds["dq"], dtype=float)
        ddq = np.asarray(ds["ddq"], dtype=float)
        tau_mat = np.asarray(ds["tau"], dtype=float)
        if q.shape[1] != nv or tau_mat.shape[1] != nv:
            raise ValueError(
                f"dataset n_joint={q.shape[1]} / tau={tau_mat.shape[1]} != model nv={nv}"
            )
        t = np.arange(q.shape[0], dtype=float) * float(ds["dt"])
        # Optional subsample (same knob as pinocchio path)
        sl = slice(None, None, args.subsample)
        t, q, dq, ddq, tau_mat = t[sl], q[sl], dq[sl], ddq[sl], tau_mat[sl]
        print(
            tr(
                f"[dynamic] loaded {args.data_path}  samples={len(t)} "
                f"(subsample={args.subsample}) traj={ds['traj_type']}",
                f"[动力学] 已加载 {args.data_path}  样本数={len(t)} "
                f"（抽稀={args.subsample}）轨迹={ds['traj_type']}",
            )
        )
        # URDF inertial as reference; friction truth unknown for external logs.
        pi_fc = np.zeros(nv)
        pi_fv = np.zeros(nv)
        pi_true = np.concatenate([pi_inertial, pi_fc, pi_fv])
        print(
            tr(
                f"[dynamic] params: inertial={pi_inertial.size}, fc={nv}, fv={nv}, "
                f"total={pi_true.size} (fc/fv truth N/A)",
                f"[动力学] 参数：惯量={pi_inertial.size}, 库仑={nv}, 粘性={nv}, "
                f"合计={pi_true.size}（外部数据无摩擦真值）",
            )
        )
        Y = _build_dynamic_Y(model, data, q, dq, ddq, K_coulomb=K_coulomb)
        tau = tau_mat.reshape(-1)
        outlier_mask = np.zeros(q.shape[0], dtype=bool)
    else:
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
        print(
            tr(
                f"[dynamic] samples={len(t)} (subsample={args.subsample})",
                f"[动力学] 样本数={len(t)}（抽稀={args.subsample}）",
            )
        )

        pi_fc = rng.uniform(1.0, 5.0, size=nv)
        pi_fv = rng.uniform(0.1, 0.5, size=nv)
        pi_true = np.concatenate([pi_inertial, pi_fc, pi_fv])
        print(
            tr(
                f"[dynamic] params: inertial={pi_inertial.size}, fc={nv}, fv={nv}, "
                f"total={pi_true.size}",
                f"[动力学] 参数：惯量={pi_inertial.size}, 库仑={nv}, 粘性={nv}, "
                f"合计={pi_true.size}",
            )
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
            K_coulomb=K_coulomb,
        )

    idx, Yb, diag = select_base_columns(Y)
    print(
        tr(
            f"[dynamic] QR: full cols={Y.shape[1]}, base={len(idx)}",
            f"[动力学] QR：全列={Y.shape[1]} → 基参数={len(idx)}",
        )
    )
    print(
        tr(
            f"          R diag head={np.array2string(diag[:10], precision=2)}",
            f"         R 对角前几项={np.array2string(diag[:10], precision=2)}",
        )
    )
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

    print(tr(f"[dynamic] method={args.method}", f"[动力学] 估计方法={args.method}"))
    print(
        tr(
            f"          torque RMSE (all)    = {rmse:.4f} N·m",
            f"         力矩 RMSE（全部样本）= {rmse:.4f} N·m  ← 含异常点，通常偏高",
        )
    )
    print(
        tr(
            f"          torque RMSE (inlier) = {rmse_in:.4f} N·m",
            f"         力矩 RMSE（内点）    = {rmse_in:.4f} N·m  ← 优先看此项",
        )
    )
    print(
        tr(
            f"          base-param relative error = {rel_param:.4f}",
            f"         基参数相对误差       = {rel_param:.4f}",
        )
    )
    if args.method == "robust_wls":
        print(
            tr(
                f"          rejected samples = {info.get('n_rejected')} / {len(t)}",
                f"         剔除样本数 = {info.get('n_rejected')} / {len(t)}",
            )
        )
        print(
            tr(
                f"          true outliers     = {int(outlier_mask.sum())}",
                f"         注入异常点数 = {int(outlier_mask.sum())}",
            )
        )

    # Cross-check: hold-out last 20% time for prediction RMSE
    n = len(t)
    n_te = max(1, n // 5)
    # rebuild Y_te quickly from stored arrays
    Y_te = Y[-n_te * nv :]
    tau_te = tau[-n_te * nv :]
    Yb_te = Y_te[:, idx]
    rmse_te = float(np.sqrt(np.mean((tau_te - Yb_te @ pi_hat) ** 2)))
    print(
        tr(
            f"          hold-out torque RMSE = {rmse_te:.4f} N·m",
            f"         留出集力矩 RMSE     = {rmse_te:.4f} N·m",
        )
    )

    if args.results_dir:
        out_dir = Path(args.results_dir)
    else:
        out_dir = Path(__file__).resolve().parents[1] / "results"
        if args.data_source == "file":
            out_dir = out_dir / "isaac_dynamic"
    out_dir.mkdir(parents=True, exist_ok=True)
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
    print(
        tr(
            f"[dynamic] saved figure -> {fig_path}",
            f"[动力学] 已保存图 → {fig_path}",
        )
    )

    np.savez(
        out_dir / f"dynamic_{args.method}.npz",
        pi_hat=pi_hat,
        pi_true_b=pi_true_b,
        idx=idx,
        rmse=rmse,
        rmse_te=rmse_te,
        rel_param=rel_param,
        joint_names=np.array(names),
        data_source=np.array(args.data_source),
    )


if __name__ == "__main__":
    main()
