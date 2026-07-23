"""Excitation trajectories: Fourier (dynamics) and slow cosine (statics)."""

from __future__ import annotations

import numpy as np


def fourier_trajectory(
    q_min: np.ndarray,
    q_max: np.ndarray,
    fs: float = 100.0,
    fundamental_freq: float = 0.1,
    n_periods: int = 10,
    harmonics: int = 5,
    amplitude_scale: float = 0.8,
    rng: np.random.Generator | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Finite Fourier series per joint with analytic q, dq, ddq.

    q_j(t) = q_mid + sum_{k=1}^H (a_{jk}/(k w) sin(k w t) - b_{jk}/(k w) cos(k w t))
    """
    rng = rng or np.random.default_rng(0)
    nv = q_min.size
    w = 2.0 * np.pi * fundamental_freq
    T = n_periods / fundamental_freq
    t = np.arange(0.0, T, 1.0 / fs)
    q_mid = 0.5 * (q_min + q_max)
    span = 0.5 * (q_max - q_min) * amplitude_scale

    a = rng.uniform(-1.0, 1.0, size=(nv, harmonics))
    b = rng.uniform(-1.0, 1.0, size=(nv, harmonics))

    # First pass: unscaled amplitudes of the oscillatory part
    q_osc = np.zeros((t.size, nv))
    dq = np.zeros_like(q_osc)
    ddq = np.zeros_like(q_osc)
    for j in range(nv):
        for k in range(1, harmonics + 1):
            wt = k * w * t
            ak, bk = a[j, k - 1], b[j, k - 1]
            q_osc[:, j] += (ak / (k * w)) * np.sin(wt) - (bk / (k * w)) * np.cos(wt)
            dq[:, j] += ak * np.cos(wt) + bk * np.sin(wt)
            ddq[:, j] += k * w * (-ak * np.sin(wt) + bk * np.cos(wt))

    # Scale so peak-to-peak fits within amplitude_scale * joint range
    for j in range(nv):
        pp = q_osc[:, j].max() - q_osc[:, j].min()
        if pp < 1e-9:
            continue
        s = (2.0 * span[j]) / pp
        q_osc[:, j] *= s
        dq[:, j] *= s
        ddq[:, j] *= s

    q = q_mid[None, :] + q_osc
    # Clip softly into limits
    q = np.clip(q, q_min + 1e-3, q_max - 1e-3)
    return t, q, dq, ddq


def cosine_static_trajectory(
    q_min: np.ndarray,
    q_max: np.ndarray,
    fs: float = 50.0,
    duration: float = 40.0,
    n_cycles: float = 2.0,
    vel_scale: float = 0.15,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Slow cosine sweeping for static (gravity + Coulomb) identification."""
    t = np.arange(0.0, duration, 1.0 / fs)
    q_mid = 0.5 * (q_min + q_max)
    amp = 0.4 * (q_max - q_min)
    w = 2.0 * np.pi * n_cycles / duration
    # phase offset per joint to enrich configurations
    phases = np.linspace(0, np.pi, q_min.size, endpoint=False)
    q = q_mid[None, :] + amp[None, :] * np.cos(w * t[:, None] + phases[None, :])
    dq = -amp[None, :] * w * np.sin(w * t[:, None] + phases[None, :]) * vel_scale
    q = np.clip(q, q_min + 1e-3, q_max - 1e-3)
    return t, q, dq
