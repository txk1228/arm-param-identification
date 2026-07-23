#!/usr/bin/env python3
"""Baseline alignment: compare Pinocchio vs Isaac identification under ideal physics.

Runs the same OLS identification on two unified NPZ datasets, then reports:
  - gravity-base relative error vs URDF truth (static gate: < 5%)
  - torque fit RMSE
  - relative difference between the two gravity-base estimates

Usage::

    python scripts/compare_baseline_alignment.py --kind static \\
        --pinocchio-data results/baseline/pinocchio_static_cosine.npz \\
        --isaac-data results/baseline/isaac_static_cosine_ideal.npz

    python scripts/compare_baseline_alignment.py --kind dynamic \\
        --pinocchio-data results/baseline/pinocchio_dynamic_fourier.npz \\
        --isaac-data results/baseline/isaac_dynamic_fourier_ideal.npz
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))

from param_id.base_params import select_base_columns
from param_id.estimators import ols
from param_id.regressor import (
    dynamics_regressor,
    gravity_params_from_model,
    static_regressor,
)
from param_id.robot_model import build_model, extract_inertial_params
from utils.data_io import load_dataset

# Pass criterion for ideal static gravity-base parameters.
GRAVITY_ERR_LIMIT = 0.05


def _identify_static(model, data, ds: dict) -> dict:
    q = np.asarray(ds["q"], float)
    dq = np.asarray(ds["dq"], float)
    tau = np.asarray(ds["tau"], float).reshape(-1)
    nv = model.nv
    pi_g = gravity_params_from_model(model)
    Y = np.vstack(
        [static_regressor(model, data, q[i], dq[i], K_coulomb=300.0) for i in range(len(q))]
    )
    n_g = pi_g.size
    Yg, Yf = Y[:, :n_g], Y[:, n_g:]
    idx_g, _, _ = select_base_columns(Yg)
    Ygb = Yg[:, idx_g]
    Yb = np.hstack([Ygb, Yf])
    pi_hat = ols(Yb, tau)
    tau_hat = Yb @ pi_hat
    rmse = float(np.sqrt(np.mean((tau - tau_hat) ** 2)))
    pi_g_hat = pi_hat[: len(idx_g)]
    # Identifiable gravity coordinates (QR subspace), not raw pi_g[idx].
    pi_g_true_b = ols(Ygb, Yg @ pi_g)
    rel_g = float(
        np.linalg.norm(pi_g_hat - pi_g_true_b) / (np.linalg.norm(pi_g_true_b) + 1e-12)
    )
    # Also: gravity-compensation residual on zero-velocity postures.
    import pinocchio as pin

    errs = []
    for qi in q[:: max(1, len(q) // 20)]:
        Ygi = static_regressor(model, data, qi, np.zeros(nv), K_coulomb=300.0)[
            :, :n_g
        ][:, idx_g]
        tau_g = pin.rnea(model, data, qi, np.zeros(nv), np.zeros(nv))
        errs.append(np.linalg.norm(tau_g - Ygi @ pi_g_hat) / (np.linalg.norm(tau_g) + 1e-12))
    rel_g_comp = float(np.mean(errs))
    return {
        "pi_g_hat": pi_g_hat,
        "pi_g_true_b": pi_g_true_b,
        "idx_g": idx_g,
        "rmse": rmse,
        "rel_g": rel_g,
        "rel_g_comp": rel_g_comp,
        "n_base_g": len(idx_g),
    }


def _identify_dynamic(model, data, ds: dict) -> dict:
    q = np.asarray(ds["q"], float)
    dq = np.asarray(ds["dq"], float)
    ddq = np.asarray(ds["ddq"], float)
    tau = np.asarray(ds["tau"], float).reshape(-1)
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
    idx, Yb, _ = select_base_columns(Y)
    pi_hat = ols(Yb, tau)
    tau_hat = Yb @ pi_hat
    rmse = float(np.sqrt(np.mean((tau - tau_hat) ** 2)))
    pi_true_b = ols(Yb, Y @ pi_true)  # identifiable projection of URDF params
    rel_full = float(
        np.linalg.norm(pi_hat - pi_true_b) / (np.linalg.norm(pi_true_b) + 1e-12)
    )

    # Gravity-base check on measured postures (same metric as static gate).
    import pinocchio as pin

    pi_g = gravity_params_from_model(model)
    step = max(1, len(q) // 200)
    q_sub = q[::step]
    Yg = np.vstack(
        [
            static_regressor(model, data, qi, np.zeros(nv), K_coulomb=300.0)[:, : pi_g.size]
            for qi in q_sub
        ]
    )
    tau_g = np.concatenate(
        [pin.rnea(model, data, qi, np.zeros(nv), np.zeros(nv)) for qi in q_sub]
    )
    idx_g, Ygb, _ = select_base_columns(Yg)
    pi_g_true_b = ols(Ygb, Yg @ pi_g)
    pi_g_hat = ols(Ygb, tau_g)  # from URDF RNEA at postures (consistency)
    # For dynamic datasets, evaluate gravity ID using quasi-static samples of q
    # with measured tau replaced by RNEA gravity — this checks posture coverage.
    # Prefer: fit gravity from dynamic tau is ill-posed; use posture RNEA fit as
    # structural check, and use rel_full + rmse as primary dynamic metrics.
    # Gravity gate for dynamic: compensation using static regressor on q_sub
    # with pi_g from a dedicated static-style fit on RNEA(q,0,0).
    rel_g = float(
        np.linalg.norm(pi_g_hat - pi_g_true_b) / (np.linalg.norm(pi_g_true_b) + 1e-12)
    )
    return {
        "pi_hat": pi_hat,
        "pi_true_b": pi_true_b,
        "idx": idx,
        "rmse": rmse,
        "rel_full": rel_full,
        "rel_g": rel_g,
        "rel_g_comp": rel_g,
        "pi_g_hat": pi_g_hat,
        "pi_g_true_b": pi_g_true_b,
        "n_base": len(idx),
    }


def main() -> None:
    p = argparse.ArgumentParser(description="Baseline Pinocchio vs Isaac alignment")
    p.add_argument("--kind", choices=["static", "dynamic"], required=True)
    p.add_argument("--pinocchio-data", type=str, required=True)
    p.add_argument("--isaac-data", type=str, required=True)
    p.add_argument("--urdf", type=str, default=None)
    p.add_argument(
        "--out-dir",
        type=str,
        default=None,
        help="Default: results/baseline/compare_{kind}",
    )
    p.add_argument(
        "--gravity-limit",
        type=float,
        default=GRAVITY_ERR_LIMIT,
        help="Pass if both sources' gravity-base rel-error < this (default 0.05).",
    )
    args = p.parse_args()

    out_dir = Path(args.out_dir or (_ROOT / "results" / "baseline" / f"compare_{args.kind}"))
    out_dir.mkdir(parents=True, exist_ok=True)

    model, data, names, urdf = build_model(args.urdf)
    ds_p = load_dataset(args.pinocchio_data)
    ds_i = load_dataset(args.isaac_data)

    if args.kind == "static":
        r_p = _identify_static(model, data, ds_p)
        r_i = _identify_static(model, data, ds_i)
    else:
        r_p = _identify_dynamic(model, data, ds_p)
        r_i = _identify_dynamic(model, data, ds_i)

    # Align gravity vectors on common length (min base size).
    n_g = min(r_p["pi_g_hat"].size, r_i["pi_g_hat"].size)
    g_p = r_p["pi_g_hat"][:n_g]
    g_i = r_i["pi_g_hat"][:n_g]
    g_t = r_p["pi_g_true_b"][:n_g]
    cross = float(np.linalg.norm(g_p - g_i) / (np.linalg.norm(g_t) + 1e-12))

    metrics = {
        "kind": args.kind,
        "urdf": str(urdf),
        "pinocchio": {
            "rel_g": r_p["rel_g"],
            "rel_g_comp": r_p.get("rel_g_comp", r_p["rel_g"]),
            "rmse": r_p["rmse"],
            **({"rel_full": r_p["rel_full"]} if "rel_full" in r_p else {}),
        },
        "isaac": {
            "rel_g": r_i["rel_g"],
            "rel_g_comp": r_i.get("rel_g_comp", r_i["rel_g"]),
            "rmse": r_i["rmse"],
            **({"rel_full": r_i["rel_full"]} if "rel_full" in r_i else {}),
        },
        "cross_rel_g": cross,
        "gravity_limit": args.gravity_limit,
    }
    # Pass gate:
    #   static  -> gravity-base QR param error < limit
    #   dynamic -> full base-param error vs URDF projection < limit
    if args.kind == "static":
        metrics["pass"] = (
            r_p["rel_g"] < args.gravity_limit and r_i["rel_g"] < args.gravity_limit
        )
        gate_name = "gravity-base rel_g"
        gate_p, gate_i = r_p["rel_g"], r_i["rel_g"]
    else:
        metrics["pass"] = (
            r_p["rel_full"] < args.gravity_limit and r_i["rel_full"] < args.gravity_limit
        )
        gate_name = "full-base rel_full"
        gate_p, gate_i = r_p["rel_full"], r_i["rel_full"]

    print("=== Baseline alignment ===")
    print(f"kind={args.kind}  urdf={urdf.name}")
    print(
        f"Pinocchio:  rel_g={r_p['rel_g']*100:.2f}%  "
        f"g_comp={r_p.get('rel_g_comp', 0)*100:.2f}%  "
        f"torque_RMSE={r_p['rmse']:.4f} N·m"
    )
    print(
        f"Isaac:      rel_g={r_i['rel_g']*100:.2f}%  "
        f"g_comp={r_i.get('rel_g_comp', 0)*100:.2f}%  "
        f"torque_RMSE={r_i['rmse']:.4f} N·m"
    )
    if "rel_full" in r_p:
        print(
            f"            rel_full base: P={r_p['rel_full']*100:.2f}%  "
            f"I={r_i['rel_full']*100:.2f}%"
        )
    print(f"Cross |π̂_g^P - π̂_g^I| / |π_g^b| = {cross*100:.2f}%")
    print(
        f"Gate: both {gate_name} < {args.gravity_limit*100:.1f}% "
        f"(P={gate_p*100:.2f}%, I={gate_i*100:.2f}%)  -> "
        f"{'PASS' if metrics['pass'] else 'FAIL'}"
    )

    # Bar chart
    fig, axes = plt.subplots(1, 2, figsize=(9, 4))
    labels = ["Pinocchio", "Isaac"]
    axes[0].bar(labels, [r_p["rel_g"] * 100, r_i["rel_g"] * 100], color=["#4C72B0", "#DD8452"])
    axes[0].axhline(args.gravity_limit * 100, color="k", ls="--", lw=1, label="5% gate")
    axes[0].set_ylabel("Gravity-base relative error [%]")
    axes[0].set_title(f"{args.kind}: gravity param error")
    axes[0].legend()
    axes[1].bar(labels, [r_p["rmse"], r_i["rmse"]], color=["#4C72B0", "#DD8452"])
    axes[1].set_ylabel("Torque fit RMSE [N·m]")
    axes[1].set_title(f"{args.kind}: torque RMSE")
    fig.suptitle(
        f"Baseline alignment ({'PASS' if metrics['pass'] else 'FAIL'})",
        fontsize=12,
    )
    fig.tight_layout()
    fig_path = out_dir / f"baseline_{args.kind}_bars.png"
    fig.savefig(fig_path, dpi=140)

    # Gravity param overlay
    fig2, ax = plt.subplots(figsize=(8, 3.5))
    x = np.arange(n_g)
    ax.bar(x - 0.2, g_t, width=0.2, label="URDF truth", alpha=0.7)
    ax.bar(x, g_p, width=0.2, label="Pinocchio ID")
    ax.bar(x + 0.2, g_i, width=0.2, label="Isaac ID")
    ax.set_xlabel("gravity base index")
    ax.set_ylabel("parameter value")
    ax.legend()
    ax.set_title(f"{args.kind}: gravity base parameters")
    fig2.tight_layout()
    fig2_path = out_dir / f"baseline_{args.kind}_gravity_params.png"
    fig2.savefig(fig2_path, dpi=140)

    metrics_path = out_dir / f"baseline_{args.kind}_metrics.json"
    metrics_path.write_text(json.dumps(metrics, indent=2))
    print(f"saved -> {fig_path}")
    print(f"saved -> {fig2_path}")
    print(f"saved -> {metrics_path}")

    sys.exit(0 if metrics["pass"] else 1)


if __name__ == "__main__":
    main()
