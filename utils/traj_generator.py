"""Trajectory generation facade for data-collection scripts.

Wraps :mod:`param_id.trajectory` so collectors can import from ``utils``
without touching the core identification package. Core algorithms stay unchanged.
"""

from __future__ import annotations

import numpy as np

from param_id.trajectory import cosine_static_trajectory, fourier_trajectory

__all__ = [
    "fourier_trajectory",
    "cosine_static_trajectory",
    "generate_excitation",
]


def generate_excitation(
    traj: str,
    q_min: np.ndarray,
    q_max: np.ndarray,
    *,
    dt: float = 0.01,
    n_periods: int = 10,
    fourier_harmonics: int = 5,
    fundamental_freq: float = 0.1,
    seed: int = 0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Build a desired excitation trajectory.

    Parameters
    ----------
    traj :
        ``"fourier"`` (dynamics) or ``"cosine"`` (statics).
    q_min, q_max :
        Joint limits, shape ``(n_joint,)``.
    dt :
        Sample period [s] (``fs = 1/dt``).
    n_periods :
        Fourier fundamental periods, or cosine slow-sweep cycles.
    fourier_harmonics :
        Number of Fourier harmonics (ignored for cosine).
    fundamental_freq :
        Fourier fundamental frequency [Hz].
    seed :
        RNG seed for Fourier amplitudes.

    Returns
    -------
    t, q_des, dq_des, ddq_des
        Arrays with shapes ``(N,)`` and ``(N, n_joint)``.
    """
    fs = 1.0 / float(dt)
    traj = traj.lower().strip()

    if traj == "fourier":
        t, q, dq, ddq = fourier_trajectory(
            q_min,
            q_max,
            fs=fs,
            fundamental_freq=fundamental_freq,
            n_periods=n_periods,
            harmonics=fourier_harmonics,
            rng=np.random.default_rng(seed),
        )
        return t, q, dq, ddq

    if traj == "cosine":
        # Map n_periods -> slow cosine duration / cycles (statics-friendly).
        duration = max(20.0, float(n_periods) / max(fundamental_freq, 1e-6))
        t, q, dq = cosine_static_trajectory(
            q_min,
            q_max,
            fs=fs,
            duration=duration,
            n_cycles=float(n_periods),
        )
        # Analytic accel for cosine: d/dt [ -A w sin(wt+phi) * vel_scale ]
        # Keep consistent with vel_scale=0.15 used inside cosine_static_trajectory.
        vel_scale = 0.15
        q_mid = 0.5 * (q_min + q_max)
        amp = 0.4 * (q_max - q_min)
        w = 2.0 * np.pi * float(n_periods) / duration
        phases = np.linspace(0, np.pi, q_min.size, endpoint=False)
        ddq = -amp[None, :] * (w**2) * np.cos(w * t[:, None] + phases[None, :]) * vel_scale
        # Prefer returned q (already clipped); recompute mid residual not needed.
        _ = q_mid  # silence unused if clipped path dominates
        return t, q, dq, ddq

    raise ValueError(f"Unknown traj={traj!r}; expected 'fourier' or 'cosine'")
