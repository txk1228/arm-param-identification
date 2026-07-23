"""Standardized NPZ dataset I/O for Pinocchio-synthesized and Isaac-collected data.

Fixed schema
------------
q, dq, ddq, tau : (N, n_joint) float arrays
dt              : float sampling step [s]
traj_type       : str trajectory label (e.g. "fourier", "cosine", "isaac_fourier")
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import numpy as np

# Required keys and expected Python / NumPy types for the unified schema.
REQUIRED_KEYS = ("q", "dq", "ddq", "tau", "dt", "traj_type")
_ARRAY_KEYS = ("q", "dq", "ddq", "tau")


def validate_dataset(data: Mapping[str, Any]) -> None:
    """Raise ValueError if ``data`` does not match the unified schema.

    Checks
    ------
    - all required keys present
    - q/dq/ddq/tau are 2-D with identical shape (N, n_joint), N >= 1
    - no NaN / Inf in the arrays
    - dt is a positive finite float
    - traj_type is a non-empty string
    """
    missing = [k for k in REQUIRED_KEYS if k not in data]
    if missing:
        raise ValueError(f"dataset missing keys: {missing}")

    arrays = []
    for key in _ARRAY_KEYS:
        arr = np.asarray(data[key])
        if arr.ndim != 2:
            raise ValueError(f"{key} must be 2-D (N, n_joint), got shape {arr.shape}")
        if arr.shape[0] < 1:
            raise ValueError(f"{key} must have N >= 1 samples, got shape {arr.shape}")
        if not np.issubdtype(arr.dtype, np.number):
            raise ValueError(f"{key} must be numeric, got dtype {arr.dtype}")
        if not np.isfinite(arr).all():
            raise ValueError(f"{key} contains NaN or Inf")
        arrays.append(arr)

    shapes = {a.shape for a in arrays}
    if len(shapes) != 1:
        detail = {k: np.asarray(data[k]).shape for k in _ARRAY_KEYS}
        raise ValueError(f"q/dq/ddq/tau shape mismatch: {detail}")

    dt = float(data["dt"])
    if not np.isfinite(dt) or dt <= 0.0:
        raise ValueError(f"dt must be a positive finite float, got {data['dt']!r}")

    traj_type = data["traj_type"]
    if not isinstance(traj_type, str) or not traj_type.strip():
        # np.savez may round-trip strings as np.str_; accept those too.
        if isinstance(traj_type, np.str_):
            traj_type = str(traj_type)
        if not isinstance(traj_type, str) or not str(traj_type).strip():
            raise ValueError(f"traj_type must be a non-empty str, got {traj_type!r}")


def save_dataset(save_path: str | Path, data_dict: Mapping[str, Any]) -> Path:
    """Validate and save a dataset to ``.npz``.

    Parameters
    ----------
    save_path :
        Output path (``.npz`` suffix added if missing).
    data_dict :
        Must contain keys: q, dq, ddq, tau, dt, traj_type.

    Returns
    -------
    Path
        Absolute path written.
    """
    validate_dataset(data_dict)

    path = Path(save_path)
    if path.suffix != ".npz":
        path = path.with_suffix(".npz")
    path.parent.mkdir(parents=True, exist_ok=True)

    # Cast arrays to float64 for stable cross-tool exchange.
    payload = {
        "q": np.asarray(data_dict["q"], dtype=np.float64),
        "dq": np.asarray(data_dict["dq"], dtype=np.float64),
        "ddq": np.asarray(data_dict["ddq"], dtype=np.float64),
        "tau": np.asarray(data_dict["tau"], dtype=np.float64),
        "dt": np.float64(data_dict["dt"]),
        "traj_type": str(data_dict["traj_type"]),
    }
    np.savez_compressed(path, **payload)
    return path.resolve()


def load_dataset(data_path: str | Path) -> dict[str, Any]:
    """Load a ``.npz`` dataset and return a plain dict after validation.

    Returns
    -------
    dict
        Keys: q, dq, ddq, tau (float64 arrays), dt (float), traj_type (str).
    """
    path = Path(data_path)
    if not path.is_file():
        raise FileNotFoundError(f"dataset not found: {path}")

    with np.load(path, allow_pickle=False) as raw:
        data = {
            "q": np.asarray(raw["q"], dtype=np.float64),
            "dq": np.asarray(raw["dq"], dtype=np.float64),
            "ddq": np.asarray(raw["ddq"], dtype=np.float64),
            "tau": np.asarray(raw["tau"], dtype=np.float64),
            "dt": float(np.asarray(raw["dt"]).reshape(())),
            "traj_type": str(raw["traj_type"]),
        }

    validate_dataset(data)
    return data
