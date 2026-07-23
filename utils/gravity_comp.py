"""Gravity torque helpers for compensation / verification (Pinocchio)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pinocchio as pin

from param_id.regressor import gravity_regressor_numeric


def load_static_id_result(path: str | Path) -> dict[str, Any]:
    """Load ``static_*.npz`` from identify_static.py."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(path)
    raw = np.load(path, allow_pickle=False)
    return {
        "pi_hat": np.asarray(raw["pi_hat"], dtype=float),
        "idx_g": np.asarray(raw["idx_g"], dtype=int),
        "pi_true_b": np.asarray(raw["pi_true_b"], dtype=float)
        if "pi_true_b" in raw.files
        else None,
    }


def gravity_torque_urdf(model: Any, data: Any, q: np.ndarray) -> np.ndarray:
    """Nominal gravity torque from URDF inertias: RNEA(q,0,0)."""
    nv = model.nv
    return pin.rnea(model, data, q, np.zeros(nv), np.zeros(nv)).copy()


def gravity_torque_identified(
    model: Any,
    data: Any,
    q: np.ndarray,
    pi_hat: np.ndarray,
    idx_g: np.ndarray,
) -> np.ndarray:
    """Gravity torque from identified base gravity params: Y_g[:,idx] @ pi_g."""
    Yg = gravity_regressor_numeric(model, data, q)
    n_b = len(idx_g)
    return Yg[:, idx_g] @ pi_hat[:n_b]


def residual_stats(residual: np.ndarray) -> dict[str, float]:
    """Aggregate residual torque norms over samples (N, nv) or list of vectors."""
    r = np.asarray(residual, dtype=float)
    if r.ndim == 1:
        norms = np.array([np.linalg.norm(r)])
    else:
        norms = np.linalg.norm(r, axis=1)
    return {
        "mean_norm": float(np.mean(norms)),
        "max_norm": float(np.max(norms)),
        "rms": float(np.sqrt(np.mean(r**2))),
    }


def evaluate_compensation_offline(
    model: Any,
    data: Any,
    q_list: np.ndarray,
    pi_hat: np.ndarray | None,
    idx_g: np.ndarray | None,
) -> dict[str, Any]:
    """Compare residual gravity error: none / URDF / identified vs RNEA truth."""
    truth = np.vstack([gravity_torque_urdf(model, data, q) for q in q_list])
    none_res = truth  # uncompensated residual = full gravity
    urdf_res = truth - truth  # perfect if truth is URDF
    out: dict[str, Any] = {
        "none": residual_stats(none_res),
        "urdf": residual_stats(urdf_res),
        "truth_rms": float(np.sqrt(np.mean(truth**2))),
    }
    if pi_hat is not None and idx_g is not None:
        id_tau = np.vstack(
            [gravity_torque_identified(model, data, q, pi_hat, idx_g) for q in q_list]
        )
        out["identified"] = residual_stats(truth - id_tau)
        out["id_vs_urdf_rms"] = float(np.sqrt(np.mean((id_tau - truth) ** 2)))
    return out
