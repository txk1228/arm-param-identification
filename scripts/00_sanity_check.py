#!/usr/bin/env python3
"""Sanity check: regressors must match RNEA (||τ - Yπ|| ≈ 0)."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))

from param_id.regressor import (  # noqa: E402
    dynamics_regressor,
    gravity_params_from_model,
    gravity_regressor_numeric,
)
from param_id.robot_model import (  # noqa: E402
    build_model,
    extract_inertial_params,
    joint_limits,
)
from utils.cli_lang import t  # noqa: E402


def main() -> None:
    model, data, names, urdf = build_model()
    q_min, q_max = joint_limits(model)
    rng = np.random.default_rng(1)
    q = rng.uniform(q_min + 0.05, q_max - 0.05)
    zero = np.zeros(model.nv)

    print(t("=== Sanity check: robot & regressor ===", "=== 自检：机器人模型与回归器 ==="))
    print(f"urdf: {urdf}")
    print(t(f"joints ({model.nv}): {names}", f"关节数 ({model.nv}): {names}"))

    tau_g = pin.rnea(model, data, q, zero, zero)
    Yg = gravity_regressor_numeric(model, data, q)
    pi_g = gravity_params_from_model(model)
    err_g = np.linalg.norm(tau_g - Yg @ pi_g)
    print(
        t(
            f"gravity regressor ||tau - Y_g pi|| = {err_g:.3e}",
            f"静力学回归残差 ||τ - Y_g π|| = {err_g:.3e}",
        )
    )

    dq = rng.uniform(-0.5, 0.5, size=model.nv)
    ddq = rng.uniform(-1.0, 1.0, size=model.nv)
    Y = dynamics_regressor(model, data, q, dq, ddq, with_friction=False)
    pi = extract_inertial_params(model)
    tau = pin.rnea(model, data, q, dq, ddq)
    err_d = np.linalg.norm(tau - Y @ pi)
    print(
        t(
            f"dynamic regressor ||tau - Y pi||   = {err_d:.3e}",
            f"动力学回归残差 ||τ - Y π||   = {err_d:.3e}",
        )
    )

    if err_g > 1e-4 or err_d > 1e-4:
        print(t("FAIL: regressor inconsistency.", "失败：回归器与 RNEA 不一致。"))
        sys.exit(1)
    print(
        t(
            "OK: regressors match RNEA.",
            "通过：回归器与 RNEA 一致（残差应接近 0）。",
        )
    )


if __name__ == "__main__":
    main()
