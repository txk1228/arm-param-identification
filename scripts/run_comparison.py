#!/usr/bin/env python3
"""Stage 6: cross-comparison experiments (Pinocchio vs Isaac, OLS vs robust WLS).

Three arms
----------
1. Baseline:      Pinocchio ideal (RNEA, no friction/noise) + OLS
2. Physics-OLS:   Isaac eng. (PD + friction + noise) + OLS
3. Physics-robust: same Isaac eng. data + robust whitened WLS

Outputs under ``results/comparison/``: config dump, NPZs, plots, conclusion.md.

Usage::

    conda activate env_isaaclab
    python scripts/run_comparison.py
    python scripts/run_comparison.py --skip-isaac   # force Pinocchio friction+noise proxy
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import textwrap
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
import yaml

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))

from param_id.base_params import select_base_columns
from param_id.estimators import ols, robust_wls
from param_id.regressor import dynamics_regressor
from param_id.robot_model import build_model, extract_inertial_params, joint_limits
from utils.collect_extras import inject_gaussian_noise
from utils.data_io import load_dataset, save_dataset
from utils.traj_generator import generate_excitation

OUT_DIR = _ROOT / "results" / "comparison"
DEFAULT_CFG = _ROOT / "configs" / "comparison_experiment.yaml"


@dataclass
class GroupResult:
    id: str
    name: str
    method: str
    rmse_per_joint: np.ndarray
    rmse_all: float
    rmse_inlier: float
    rel_param: float
    t: np.ndarray
    tau_meas: np.ndarray  # (N, nv)
    tau_pred: np.ndarray
    pi_hat: np.ndarray
    pi_true_b: np.ndarray
    data_source_note: str
    n_inlier: int = 0
    n_samples: int = 0


def _load_cfg(path: Path) -> dict[str, Any]:
    with path.open() as f:
        return yaml.safe_load(f)


def _export_pinocchio_ideal(cfg: dict, urdf: str | None, out_npz: Path) -> Path:
    traj = cfg["trajectory"]
    model, data, _, _ = build_model(urdf)
    q_min, q_max = joint_limits(model)
    t, q, dq, ddq = generate_excitation(
        cfg["traj"],
        q_min,
        q_max,
        dt=traj["dt"],
        n_periods=traj["n_periods"],
        fourier_harmonics=traj["fourier_harmonics"],
        fundamental_freq=traj["fundamental_freq"],
        seed=traj["seed"],
    )
    tau = np.zeros_like(q)
    for i in range(len(t)):
        tau[i] = pin.rnea(model, data, q[i], dq[i], ddq[i])
    return save_dataset(
        out_npz,
        {
            "q": q,
            "dq": dq,
            "ddq": ddq,
            "tau": tau,
            "dt": float(traj["dt"]),
            "traj_type": "pinocchio_ideal_fourier",
        },
    )


def _emulate_isaac_eng(cfg: dict, urdf: str | None, out_npz: Path) -> Path:
    """Physics-like proxy: RNEA + Coulomb/viscous + noise + sparse outliers."""
    traj = cfg["trajectory"]
    isa = cfg["isaac"]
    emu = cfg["emulation"]
    rng = np.random.default_rng(int(traj["seed"]) + 7)
    model, data, _, _ = build_model(urdf)
    q_min, q_max = joint_limits(model)
    nv = model.nv
    t, q, dq, ddq = generate_excitation(
        cfg["traj"],
        q_min,
        q_max,
        dt=traj["dt"],
        n_periods=traj["n_periods"],
        fourier_harmonics=traj["fourier_harmonics"],
        fundamental_freq=traj["fundamental_freq"],
        seed=traj["seed"],
    )
    fc = float(emu["fc"]) * np.ones(nv)
    fv = float(emu["fv"]) * np.ones(nv)
    tau = np.zeros_like(q)
    for i in range(len(t)):
        tau_id = pin.rnea(model, data, q[i], dq[i], ddq[i])
        tau_f = fc * np.tanh(50.0 * dq[i]) + fv * dq[i]
        tau[i] = tau_id + tau_f
    tau = inject_gaussian_noise(tau, float(isa["tau_noise_std"]), rng)
    q_n = inject_gaussian_noise(q, float(isa["q_noise_std"]), rng)
    # sparse torque outliers (sensor glitches)
    n_out = int(len(t) * float(emu["outlier_ratio"]))
    if n_out > 0:
        idx = rng.choice(len(t), size=n_out, replace=False)
        for i in idx:
            j = int(rng.integers(0, nv))
            tau[i, j] += rng.choice([-1.0, 1.0]) * float(emu["outlier_scale"]) * rng.uniform(
                0.5, 1.5
            )
    return save_dataset(
        out_npz,
        {
            "q": q_n,
            "dq": dq,
            "ddq": ddq,
            "tau": tau,
            "dt": float(traj["dt"]),
            "traj_type": "isaac_emulated_pd_fric_noise",
        },
    )


def _try_collect_isaac(cfg: dict, urdf: str | None, out_npz: Path) -> bool:
    """Launch collect_data_isaaclab.py; return True on success."""
    traj = cfg["trajectory"]
    isa = cfg["isaac"]
    cmd = [
        sys.executable,
        str(_ROOT / "scripts" / "collect_data_isaaclab.py"),
        "--mode",
        "dynamic",
        "--traj",
        str(cfg["traj"]),
        "--n-periods",
        str(traj["n_periods"]),
        "--fourier-harmonics",
        str(traj["fourier_harmonics"]),
        "--fundamental-freq",
        str(traj["fundamental_freq"]),
        "--dt",
        str(traj["dt"]),
        "--seed",
        str(traj["seed"]),
        "--ddq-mode",
        str(isa["ddq_mode"]),
        "--control-mode",
        str(isa["control_mode"]),
        "--kp",
        str(isa["kp"]),
        "--kd",
        str(isa["kd"]),
        "--q-noise-std",
        str(isa["q_noise_std"]),
        "--tau-noise-std",
        str(isa["tau_noise_std"]),
        "--warmup-steps",
        str(isa["warmup_steps"]),
        "--save-path",
        str(out_npz),
    ]
    if isa.get("enable_friction"):
        cmd += [
            "--enable-friction",
            "--friction-static",
            str(isa["friction_static"]),
            "--friction-coulomb",
            str(isa["friction_coulomb"]),
            "--friction-viscous",
            str(isa["friction_viscous"]),
        ]
    if isa.get("headless", True):
        cmd.append("--headless")
    if urdf:
        cmd += ["--urdf", urdf]
    print("[comparison] launching Isaac collect:\n ", " ".join(cmd))
    try:
        proc = subprocess.run(
            cmd,
            cwd=str(_ROOT),
            timeout=600,
            check=False,
        )
    except (subprocess.TimeoutExpired, OSError) as e:
        print(f"[comparison] Isaac collect failed: {e}")
        return False
    ok = proc.returncode == 0 and out_npz.is_file()
    if not ok:
        print(f"[comparison] Isaac collect exit={proc.returncode}, npz exists={out_npz.is_file()}")
    return ok


def _identify(
    model: pin.Model,
    data: pin.Data,
    ds: dict,
    method: str,
    subsample: int,
) -> dict[str, Any]:
    q = np.asarray(ds["q"], float)
    dq = np.asarray(ds["dq"], float)
    ddq = np.asarray(ds["ddq"], float)
    tau_mat = np.asarray(ds["tau"], float)
    dt = float(ds["dt"])
    sl = slice(None, None, max(1, subsample))
    q, dq, ddq, tau_mat = q[sl], dq[sl], ddq[sl], tau_mat[sl]
    t = np.arange(q.shape[0], dtype=float) * dt * max(1, subsample)
    nv = model.nv
    pi_true = np.concatenate(
        [extract_inertial_params(model), np.zeros(nv), np.zeros(nv)]
    )
    Y = np.vstack(
        [
            dynamics_regressor(model, data, q[i], dq[i], ddq[i], K_coulomb=300.0)
            for i in range(len(q))
        ]
    )
    tau = tau_mat.reshape(-1)
    idx, Yb, _ = select_base_columns(Y)
    pi_true_b = ols(Yb, Y @ pi_true)  # QR-subspace projection of URDF params

    if method == "ols":
        pi_hat = ols(Yb, tau)
    elif method == "robust_wls":
        pi_hat, _info = robust_wls(Yb, tau, nv=nv)
    else:
        raise ValueError(method)

    tau_hat = (Yb @ pi_hat).reshape(-1, nv)
    resid = tau_mat - tau_hat
    rmse_j = np.sqrt(np.mean(resid**2, axis=0))
    rmse_all = float(np.sqrt(np.mean(resid**2)))
    # Inlier RMSE: discard samples whose joint-RMS residual exceeds 2.795 * MAD
    sample_rms = np.sqrt(np.mean(resid**2, axis=1))
    med = float(np.median(sample_rms))
    mad = float(np.median(np.abs(sample_rms - med))) + 1e-12
    inlier = sample_rms <= (med + 2.795 * 1.4826 * mad)
    if not np.any(inlier):
        inlier = np.ones(len(sample_rms), dtype=bool)
    rmse_in = float(np.sqrt(np.mean(resid[inlier] ** 2)))
    rmse_j_in = np.sqrt(np.mean(resid[inlier] ** 2, axis=0))
    rel = float(
        np.linalg.norm(pi_hat - pi_true_b) / (np.linalg.norm(pi_true_b) + 1e-12)
    )
    return {
        "t": t,
        "tau_meas": tau_mat,
        "tau_pred": tau_hat,
        "rmse_per_joint": rmse_j,
        "rmse_per_joint_inlier": rmse_j_in,
        "rmse_all": rmse_all,
        "rmse_inlier": rmse_in,
        "n_inlier": int(inlier.sum()),
        "n_samples": int(len(inlier)),
        "rel_param": rel,
        "pi_hat": pi_hat,
        "pi_true_b": pi_true_b,
        "idx": idx,
    }


def _plot_rmse_bars(results: list[GroupResult], path: Path, joint_names: list[str]) -> None:
    nv = results[0].rmse_per_joint.size
    x = np.arange(nv)
    width = 0.25
    fig, ax = plt.subplots(figsize=(10, 4.2))
    colors = ["#4C72B0", "#DD8452", "#55A868"]
    short = ["Baseline OLS", "Eng. OLS", "Eng. robust WLS"]
    for k, r in enumerate(results):
        ax.bar(x + (k - 1) * width, r.rmse_per_joint, width, label=short[k], color=colors[k])
    ax.set_xticks(x)
    ax.set_xticklabels([f"j{i}" for i in range(nv)])
    ax.set_ylabel("Torque RMSE [N·m]")
    ax.set_title("Per-joint torque fit RMSE (all samples)")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def _plot_rel_param(results: list[GroupResult], path: Path) -> None:
    fig, ax = plt.subplots(figsize=(7, 4))
    labels = ["Baseline\nOLS", "Eng.\nOLS", "Eng.\nrobust WLS"]
    vals = [r.rel_param * 100 for r in results]
    colors = ["#4C72B0", "#DD8452", "#55A868"]
    ax.bar(labels, vals, color=colors)
    for i, v in enumerate(vals):
        ax.text(i, v + max(vals) * 0.02 + 1e-6, f"{v:.1f}%", ha="center", fontsize=9)
    ax.set_ylabel("Base-param relative error [%]")
    ax.set_title("Base parameter error vs URDF (QR subspace)")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def _plot_torque_curves(results: list[GroupResult], path: Path, joint: int = 0) -> None:
    titles = ["Baseline: Pinocchio ideal + OLS", "Eng.: PD/fric/noise + OLS", "Eng.: PD/fric/noise + robust WLS"]
    fig, axes = plt.subplots(3, 1, figsize=(10, 7.5), sharex=True)
    for ax, r, title in zip(axes, results, titles):
        n = len(r.t)
        sl = slice(0, min(n, 400))
        ax.plot(r.t[sl], r.tau_meas[sl, joint], lw=0.9, label="meas", color="0.35")
        ax.plot(r.t[sl], r.tau_pred[sl, joint], lw=0.9, label="pred", color="#C44E52")
        ax.set_ylabel(f"tau{joint} [N·m]")
        ax.set_title(f"{title}  (RMSE={r.rmse_per_joint[joint]:.3f}, inlier={r.rmse_inlier:.3f})")
        ax.legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("t [s]")
    fig.suptitle(f"Joint {joint} torque fit — three arms", fontsize=12)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def _write_conclusion(results: list[GroupResult], cfg: dict, path: Path, isaac_note: str) -> None:
    base, phys, rob = results
    better_rmse = "robust_wls" if rob.rmse_inlier < phys.rmse_inlier else "ols"
    lines = [
        "# Stage 6 交叉对比实验结论",
        "",
        f"- 数据说明：{isaac_note}",
        f"- 轨迹：{cfg['traj']}，n_periods={cfg['trajectory']['n_periods']}，"
        f"dt={cfg['trajectory']['dt']}，seed={cfg['trajectory']['seed']}",
        f"- Isaac 工程参数：PD(kp={cfg['isaac']['kp']}, kd={cfg['isaac']['kd']})，"
        f"摩擦(μ_s={cfg['isaac']['friction_static']}, μ_d={cfg['isaac']['friction_coulomb']}, "
        f"c_v={cfg['isaac']['friction_viscous']})，"
        f"噪声(q_std={cfg['isaac']['q_noise_std']}, τ_std={cfg['isaac']['tau_noise_std']})",
        "",
        "## 关键误差",
        "",
        "| 组别 | 方法 | RMSE全部 | RMSE内点 | 基参数相对误差 |",
        "|------|------|---------:|---------:|---------------:|",
    ]
    for r in results:
        lines.append(
            f"| {r.name} | {r.method} | {r.rmse_all:.4f} | {r.rmse_inlier:.4f} | {r.rel_param*100:.2f}% |"
        )
    lines += [
        "",
        "## 定性分析",
        "",
        f"1. **基准组**在无摩擦/无噪声的 Pinocchio 理想力矩上，OLS 扭矩 RMSE="
        f"{base.rmse_all:.4e} N·m、基参数误差 {base.rel_param*100:.2f}%，"
        "用于确认回归器与求解链路本身正确。",
        f"2. **物理仿真组**引入 PD 跟踪、关节摩擦、传感器噪声与稀疏异常后，OLS 全部样本 RMSE="
        f"{phys.rmse_all:.4f} N·m（内点 {phys.rmse_inlier:.4f}），基参数误差 {phys.rel_param*100:.2f}%，"
        "反映非理想采集对普通最小二乘的冲击。",
        f"3. **鲁棒验证组**在同一工程数据上使用白化 Huber-WLS：全部 RMSE={rob.rmse_all:.4f}，"
        f"内点 RMSE={rob.rmse_inlier:.4f}（相对物理 OLS 内点 "
        f"{'更低' if rob.rmse_inlier < phys.rmse_inlier else '接近/略高'}），"
        f"基参数误差 {rob.rel_param*100:.2f}%。"
        f"以**内点拟合**为工程指标时，更优者为 **{better_rmse}**。",
        "4. 分关节 RMSE 图可定位远端关节是否因摩擦/噪声放大而更难辨识；"
        "单关节力矩曲线用于直观对比 outliers 与鲁棒回归的差异。",
        "",
        "## 阅读顺序建议",
        "",
        "- 先看基准组确认方法正确性，再看物理组理解误差来源，最后用鲁棒组对比内点拟合。",
        "- 数据层（Isaac / 代理）与算法层（Pinocchio 回归 / 稳健估计）解耦，"
        "同一 NPZ 接口可切换辨识器。",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    p = argparse.ArgumentParser(description="Stage-6 cross comparison runner")
    p.add_argument("--config", type=str, default=str(DEFAULT_CFG))
    p.add_argument("--urdf", type=str, default=None)
    p.add_argument(
        "--skip-isaac",
        action="store_true",
        help="Skip Isaac Sim; use Pinocchio friction+noise emulation.",
    )
    p.add_argument(
        "--force-isaac",
        action="store_true",
        help="Fail if Isaac collection does not succeed (no emulation fallback).",
    )
    args = p.parse_args()

    cfg = _load_cfg(Path(args.config))
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    # Persist config used for this run
    shutil.copy(args.config, OUT_DIR / "experiment_config.yaml")

    model, data, names, urdf_path = build_model(args.urdf)
    print(f"[comparison] urdf={urdf_path.name}  joints={names}")

    pin_npz = OUT_DIR / "data_pinocchio_ideal.npz"
    isa_npz = OUT_DIR / "data_isaac_eng.npz"

    print("[comparison] (1/3) Pinocchio ideal dataset ...")
    _export_pinocchio_ideal(cfg, args.urdf, pin_npz)

    isaac_note = ""
    if args.skip_isaac:
        print("[comparison] (2/3) Emulating Isaac eng. dataset (--skip-isaac) ...")
        _emulate_isaac_eng(cfg, args.urdf, isa_npz)
        isaac_note = "Isaac 数据由 Pinocchio+摩擦/噪声仿真代理生成（--skip-isaac）"
    else:
        print("[comparison] (2/3) Isaac eng. collection ...")
        ok = _try_collect_isaac(cfg, args.urdf, isa_npz)
        if ok:
            isaac_note = "Isaac Lab 实采（PD + 摩擦 + 噪声）"
        elif args.force_isaac:
            raise RuntimeError("Isaac collection failed and --force-isaac was set")
        else:
            print("[comparison] fallback: emulating Isaac eng. dataset ...")
            _emulate_isaac_eng(cfg, args.urdf, isa_npz)
            isaac_note = (
                "Isaac Sim 启动/采集失败，已自动回退为 Pinocchio+摩擦/噪声代理数据"
                "（参数与 configs/comparison_experiment.yaml 中 isaac/emulation 一致）"
            )

    ds_pin = load_dataset(pin_npz)
    ds_isa = load_dataset(isa_npz)
    subsample = int(cfg["trajectory"].get("subsample", 2))

    print("[comparison] (3/3) Identify three arms ...")
    group_specs = [
        ("baseline", "基准组 Pinocchio理想+OLS", "ols", ds_pin, "pinocchio_ideal"),
        ("phys_ols", "物理仿真组 工程数据+OLS", "ols", ds_isa, "isaac_eng"),
        ("phys_robust", "鲁棒验证组 工程数据+robust_WLS", "robust_wls", ds_isa, "isaac_eng"),
    ]
    results: list[GroupResult] = []
    for gid, gname, method, ds, note in group_specs:
        idres = _identify(model, data, ds, method, subsample)
        results.append(
            GroupResult(
                id=gid,
                name=gname,
                method=method,
                rmse_per_joint=idres["rmse_per_joint"],
                rmse_all=idres["rmse_all"],
                rmse_inlier=idres["rmse_inlier"],
                rel_param=idres["rel_param"],
                t=idres["t"],
                tau_meas=idres["tau_meas"],
                tau_pred=idres["tau_pred"],
                pi_hat=idres["pi_hat"],
                pi_true_b=idres["pi_true_b"],
                data_source_note=note,
                n_inlier=idres["n_inlier"],
                n_samples=idres["n_samples"],
            )
        )
        np.savez(
            OUT_DIR / f"result_{gid}.npz",
            rmse_per_joint=idres["rmse_per_joint"],
            rmse_all=idres["rmse_all"],
            rmse_inlier=idres["rmse_inlier"],
            rel_param=idres["rel_param"],
            pi_hat=idres["pi_hat"],
            pi_true_b=idres["pi_true_b"],
            idx=idres["idx"],
            method=np.array(method),
        )
        print(
            f"  [{gid}] method={method}  RMSE={idres['rmse_all']:.4f}  "
            f"inlier={idres['rmse_inlier']:.4f}  rel_param={idres['rel_param']*100:.2f}%"
        )

    _plot_rmse_bars(results, OUT_DIR / "fig_rmse_per_joint.png", names)
    _plot_rel_param(results, OUT_DIR / "fig_rel_param.png")
    _plot_torque_curves(results, OUT_DIR / "fig_torque_joint0.png", joint=0)
    _write_conclusion(results, cfg, OUT_DIR / "conclusion.md", isaac_note)

    summary = {
        "urdf": str(urdf_path),
        "isaac_note": isaac_note,
        "groups": {
            r.id: {
                "name": r.name,
                "method": r.method,
                "rmse_all": r.rmse_all,
                "rmse_inlier": r.rmse_inlier,
                "rel_param": r.rel_param,
                "rmse_per_joint": r.rmse_per_joint.tolist(),
            }
            for r in results
        },
    }
    (OUT_DIR / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"[comparison] done -> {OUT_DIR}")
    print(f"[comparison] conclusion -> {OUT_DIR / 'conclusion.md'}")


if __name__ == "__main__":
    main()
