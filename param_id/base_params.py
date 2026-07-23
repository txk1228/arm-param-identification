"""Column-pivoted QR to extract numerically identifiable base parameters."""

from __future__ import annotations

import numpy as np
from scipy.linalg import qr


def select_base_columns(
    Y: np.ndarray,
    tol_ratio: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Pivoted QR on Y (n_samples*nv, n_params) -> independent columns.

    Returns
    -------
    idx : indices of base columns (sorted by pivot order, then sorted ascending)
    Yb  : Y[:, idx]
    R_diag : diagonal of R for inspection
    """
    if Y.ndim != 2:
        raise ValueError("Y must be 2D")
    # Economy pivoted QR: Y P = Q R
    Q, R, piv = qr(Y, mode="economic", pivoting=True)
    diag = np.abs(np.diag(R))
    if diag.size == 0:
        return np.array([], dtype=int), Y[:, :0], diag
    thresh = tol_ratio * diag[0]
    rank = int(np.sum(diag > thresh))
    idx = np.sort(piv[:rank])
    return idx, Y[:, idx], diag


def map_full_to_base(pi_full: np.ndarray, idx: np.ndarray) -> np.ndarray:
    return pi_full[idx]


def reconstruct_full_from_base(
    pi_base: np.ndarray,
    idx: np.ndarray,
    n_full: int,
) -> np.ndarray:
    """Pad base params back into full vector (non-base = 0). For visualization only."""
    out = np.zeros(n_full)
    out[idx] = pi_base
    return out
