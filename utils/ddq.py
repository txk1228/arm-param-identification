"""Acceleration (ddq) helpers for identification datasets.

Supports:
- ideal: use planned ``ddq_des``
- measured: central difference on ``dq`` + centered moving-average filter
"""

from __future__ import annotations

import numpy as np


def moving_average(x: np.ndarray, window: int) -> np.ndarray:
    """Centered moving average along axis 0 (edge-replicated padding)."""
    if window < 1:
        raise ValueError(f"window must be >= 1, got {window}")
    if window == 1:
        return np.asarray(x, dtype=np.float64).copy()
    if window % 2 == 0:
        window += 1  # force odd so the window is centered
    pad = window // 2
    x = np.asarray(x, dtype=np.float64)
    xp = np.pad(x, ((pad, pad), (0, 0)), mode="edge")
    kernel = np.ones(window, dtype=np.float64) / float(window)
    out = np.empty_like(x, dtype=np.float64)
    for j in range(x.shape[1]):
        out[:, j] = np.convolve(xp[:, j], kernel, mode="valid")
    return out


def central_diff_ddq(dq: np.ndarray, dt: float) -> np.ndarray:
    """Central-difference acceleration from velocity samples.

    Interior: ``(dq[i+1] - dq[i-1]) / (2 dt)``;
    ends use forward / backward first-order differences.
    """
    if dt <= 0.0:
        raise ValueError(f"dt must be positive, got {dt}")
    dq = np.asarray(dq, dtype=np.float64)
    n = dq.shape[0]
    ddq = np.zeros_like(dq)
    if n <= 1:
        return ddq
    ddq[0] = (dq[1] - dq[0]) / dt
    ddq[-1] = (dq[-1] - dq[-2]) / dt
    if n > 2:
        ddq[1:-1] = (dq[2:] - dq[:-2]) / (2.0 * dt)
    return ddq


def compute_ddq(
    ddq_mode: str,
    *,
    ddq_des: np.ndarray,
    dq_meas: np.ndarray,
    dt: float,
    ma_window: int = 5,
) -> np.ndarray:
    """Build ddq according to ``ddq_mode``.

    Parameters
    ----------
    ddq_mode :
        ``"ideal"`` — copy ``ddq_des`` (baseline alignment).
        ``"measured"`` — central diff on ``dq_meas`` + MA filter (hardware-like).
    """
    mode = ddq_mode.lower().strip()
    dq_meas = np.asarray(dq_meas, dtype=np.float64)
    if mode == "ideal":
        ddq = np.asarray(ddq_des, dtype=np.float64).copy()
        if ddq.shape != dq_meas.shape:
            raise ValueError(
                f"ddq_des shape {ddq.shape} != dq_meas shape {dq_meas.shape}"
            )
        return ddq
    if mode == "measured":
        ddq_raw = central_diff_ddq(dq_meas, dt)
        return moving_average(ddq_raw, ma_window)
    raise ValueError(f"Unknown ddq_mode={ddq_mode!r}; expected 'ideal' or 'measured'")
