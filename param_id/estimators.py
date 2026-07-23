"""OLS / Huber-IRLS / whitened robust WLS estimators."""

from __future__ import annotations

import numpy as np


def column_normalize(Y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    scales = np.linalg.norm(Y, axis=0)
    scales = np.where(scales < 1e-12, 1.0, scales)
    return Y / scales, scales


def ols(Y: np.ndarray, tau: np.ndarray) -> np.ndarray:
    Yn, scales = column_normalize(Y)
    x, *_ = np.linalg.lstsq(Yn, tau, rcond=None)
    return x / scales


def _mad_scale(r: np.ndarray) -> float:
    med = np.median(r)
    mad = np.median(np.abs(r - med))
    # Consistent with Gaussian: sigma ≈ 1.4826 * MAD
    return max(1.4826 * mad, 1e-8)


def huber_weights(r: np.ndarray, k: float = 1.345) -> np.ndarray:
    s = _mad_scale(r)
    u = np.abs(r) / s
    w = np.ones_like(r)
    mask = u > k
    w[mask] = k / u[mask]
    return w


def irls_huber(
    Y: np.ndarray,
    tau: np.ndarray,
    k: float = 1.345,
    max_iter: int = 30,
    tol: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray]:
    """Iteratively reweighted LS with Huber weights. Returns (pi, weights)."""
    Yn, scales = column_normalize(Y)
    pi = np.linalg.lstsq(Yn, tau, rcond=None)[0]
    w = np.ones(tau.shape[0])
    for _ in range(max_iter):
        r = tau - Yn @ pi
        w = huber_weights(r, k=k)
        sw = np.sqrt(w)
        pi_new = np.linalg.lstsq(Yn * sw[:, None], tau * sw, rcond=None)[0]
        if np.linalg.norm(pi_new - pi) <= tol * (1.0 + np.linalg.norm(pi)):
            pi = pi_new
            break
        pi = pi_new
    return pi / scales, w


def _whiten_matrix(Omega: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    """Omega^{-1/2} via eigendecomposition."""
    Omega = 0.5 * (Omega + Omega.T)
    eigvals, eigvecs = np.linalg.eigh(Omega)
    eigvals = np.clip(eigvals, eps, None)
    return eigvecs @ np.diag(1.0 / np.sqrt(eigvals)) @ eigvecs.T


def robust_wls(
    Y: np.ndarray,
    tau: np.ndarray,
    nv: int,
    k_huber: float = 1.345,
    hard_thresh: float = 2.795,
    max_inner: int = 20,
    max_outer: int = 10,
    tol: float = 1e-5,
) -> tuple[np.ndarray, dict]:
    """Two-layer robust WLS used in the internship PDF.

    Samples are shaped as stacked per-timestep joint torques: tau length = N * nv.
    Inner: whiten by residual covariance Omega, Huber-weight, iterate.
    Outer: hard-reject samples whose whitened residual norm exceeds hard_thresh.
    """
    n = tau.shape[0]
    assert n % nv == 0, "tau length must be multiple of nv"
    n_samples = n // nv

    Yn, scales = column_normalize(Y)
    sample_w = np.ones(n_samples)  # outer hard reject
    pi = np.linalg.lstsq(Yn, tau, rcond=None)[0]
    info = {"outer_rejected": [], "inner_iters": []}

    for outer in range(max_outer):
        # Expand sample weights to all joint rows
        w_row = np.repeat(sample_w, nv)
        keep = w_row > 0
        if keep.sum() < Yn.shape[1]:
            break

        pi_prev_outer = pi.copy()
        Omega = np.eye(nv)
        inner_count = 0
        for inner in range(max_inner):
            Wsqrt = _whiten_matrix(Omega)
            # Apply whitening block-wise
            Yw = np.zeros_like(Yn)
            tw = np.zeros_like(tau)
            for i in range(n_samples):
                sl = slice(i * nv, (i + 1) * nv)
                Yw[sl] = Wsqrt @ Yn[sl]
                tw[sl] = Wsqrt @ tau[sl]

            # Huber on whitened residual (per scalar row), times outer mask
            r = tw - Yw @ pi
            wh = huber_weights(r, k=k_huber) * w_row
            sw = np.sqrt(np.clip(wh, 0.0, None))
            pi_new = np.linalg.lstsq(Yw * sw[:, None], tw * sw, rcond=None)[0]

            # Update Omega from residuals in original (column-normalized) space
            r_orig = (tau - Yn @ pi_new).reshape(n_samples, nv)
            # Only kept samples
            mask_s = sample_w > 0
            Rk = r_orig[mask_s]
            if Rk.shape[0] < 2:
                pi = pi_new
                break
            Omega_new = (Rk.T @ Rk) / Rk.shape[0]
            # Stabilize
            Omega_new = Omega_new + 1e-8 * np.eye(nv)
            cond = np.linalg.cond(Omega_new)
            if not np.isfinite(cond) or cond > 1e12:
                pi = pi_new
                break
            rel = np.linalg.norm(pi_new - pi) / (1.0 + np.linalg.norm(pi))
            pi = pi_new
            Omega = Omega_new
            inner_count = inner + 1
            if rel < tol:
                break

        info["inner_iters"].append(inner_count)

        # Outer hard reject: any joint's whitened residual exceeds threshold
        # (PDF uses 2.795; applied per-joint, matching single-joint outlier injection)
        Wsqrt = _whiten_matrix(Omega)
        rejected = 0
        for i in range(n_samples):
            if sample_w[i] == 0:
                continue
            sl = slice(i * nv, (i + 1) * nv)
            rw = Wsqrt @ (tau[sl] - Yn[sl] @ pi)
            if np.max(np.abs(rw)) > hard_thresh:
                sample_w[i] = 0.0
                rejected += 1
        info["outer_rejected"].append(rejected)
        if rejected == 0:
            break
        if np.linalg.norm(pi - pi_prev_outer) < tol * (1.0 + np.linalg.norm(pi)):
            # still continue if new rejects happened
            pass

    info["sample_weights"] = sample_w
    info["n_rejected"] = int(np.sum(sample_w == 0))
    return pi / scales, info
