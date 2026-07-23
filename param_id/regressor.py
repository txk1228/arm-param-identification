"""Regressors: gravity (finite-diff RNEA), friction, full dynamics."""

from __future__ import annotations

import numpy as np
import pinocchio as pin


def coulomb_friction_regressor(dq: np.ndarray, K: float = 300.0) -> np.ndarray:
    """Smooth Coulomb friction: (2/pi) * arctan(K * dq) per joint -> diag row."""
    return (2.0 / np.pi) * np.arctan(K * dq)


def viscous_friction_regressor(dq: np.ndarray) -> np.ndarray:
    return dq.copy()


def gravity_regressor_numeric(
    model: pin.Model,
    data: pin.Data,
    q: np.ndarray,
    eps: float = 1e-9,
) -> np.ndarray:
    """Build Y_g(q) by RNEA with one-body-at-a-time unit inertias.

    Pinocchio has no analytic gravity regressor. Procedure (PDF §静力学):
      1. Zero all body inertias (including universe / fixed fused mass).
      2. For each body i and each of [m, mx, my, mz], place a unit parameter,
         run RNEA(q,0,0); that torque vector is one column of Y_g.
      3. Restore inertias.

    Body index i runs over joints 1..njoints-1 (same ordering as dynamics
    regressor bodies). Universe inertia is left zero so it does not appear
    in Y_g (unidentifiable / not needed for arm gravity compensation).
    """
    nv = model.nv
    n_bodies = model.njoints - 1  # skip universe
    Y = np.zeros((nv, 4 * n_bodies))
    inertias_bak = [model.inertias[i].copy() for i in range(model.njoints)]
    zero_v = np.zeros(nv)
    I0 = pin.Inertia.Zero()

    def clear_all() -> None:
        for i in range(model.njoints):
            model.inertias[i] = I0.copy()

    def rnea_g() -> np.ndarray:
        return pin.rnea(model, data, q, zero_v, zero_v).copy()

    col = 0
    for i in range(1, model.njoints):
        clear_all()
        model.inertias[i] = pin.Inertia(1.0, np.zeros(3), np.zeros((3, 3)))
        tau_m = rnea_g()
        Y[:, col] = tau_m
        col += 1
        for axis in range(3):
            clear_all()
            c = np.zeros(3)
            c[axis] = 1.0
            model.inertias[i] = pin.Inertia(1.0, c, eps * np.eye(3))
            # tau(m=1,c=e_a) = Y_m + Y_{mc_a}  =>  Y_{mc_a} = tau - Y_m
            Y[:, col] = rnea_g() - tau_m
            col += 1

    for i in range(model.njoints):
        model.inertias[i] = inertias_bak[i]
    return Y


def gravity_params_from_model(model: pin.Model) -> np.ndarray:
    """True static gravity params [m, mx, my, mz] per body."""
    out = []
    for i in range(1, model.njoints):
        iner = model.inertias[i]
        m = iner.mass
        c = iner.lever
        out.extend([m, m * c[0], m * c[1], m * c[2]])
    return np.asarray(out, dtype=float)


def dynamics_regressor(
    model: pin.Model,
    data: pin.Data,
    q: np.ndarray,
    dq: np.ndarray,
    ddq: np.ndarray,
    K_coulomb: float = 300.0,
    with_friction: bool = True,
) -> np.ndarray:
    """Y_dyn | Y_fc | Y_fv  — full torque regressor row-block for one sample."""
    Y_inertial = pin.computeJointTorqueRegressor(model, data, q, dq, ddq)
    if not with_friction:
        return Y_inertial
    fc = coulomb_friction_regressor(dq, K=K_coulomb)
    fv = viscous_friction_regressor(dq)
    # Each friction param is per-joint diagonal -> columns are e_j * f_j(qdot)
    Y_fc = np.diag(fc)
    Y_fv = np.diag(fv)
    return np.hstack([Y_inertial, Y_fc, Y_fv])


def static_regressor(
    model: pin.Model,
    data: pin.Data,
    q: np.ndarray,
    dq: np.ndarray,
    K_coulomb: float = 300.0,
) -> np.ndarray:
    """[Y_g(q) | Y_fc(dq)] for static identification."""
    Yg = gravity_regressor_numeric(model, data, q)
    fc = coulomb_friction_regressor(dq, K=K_coulomb)
    Yf = np.diag(fc)
    return np.hstack([Yg, Yf])
