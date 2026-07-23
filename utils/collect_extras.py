"""Control / sensing helpers for Isaac collection (offline-testable)."""

from __future__ import annotations

import numpy as np


def joint_pd_torque(
    q: np.ndarray,
    dq: np.ndarray,
    q_des: np.ndarray,
    dq_des: np.ndarray,
    kp: float | np.ndarray,
    kd: float | np.ndarray,
) -> np.ndarray:
    """Joint-space PD: ``tau = Kp (q_des - q) + Kd (dq_des - dq)``."""
    return kp * (q_des - q) + kd * (dq_des - dq)


def inject_gaussian_noise(
    x: np.ndarray,
    std: float,
    rng: np.random.Generator,
) -> np.ndarray:
    """Add i.i.d. Gaussian noise; ``std <= 0`` returns a copy unchanged."""
    x = np.asarray(x, dtype=np.float64)
    if std is None or std <= 0.0:
        return x.copy()
    return x + rng.normal(0.0, float(std), size=x.shape)
