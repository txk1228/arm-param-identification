"""Load a 7-DoF identification model from URDF."""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import pinocchio as pin

ROOT = Path(__file__).resolve().parents[1]
DEMO_URDF = ROOT / "models" / "demo_7dof" / "demo_arm.urdf"
PROPRIETARY_LEFT_URDF = (
    ROOT / "assets" / "proprietary" / "urdf" / "i7_left_arm.urdf"
)


def resolve_urdf(urdf: str | Path | None = None) -> Path:
    """Resolve URDF path.

    Priority:
      1. explicit ``urdf`` argument
      2. env ``PARAM_ID_URDF``
      3. proprietary left-arm URDF if present locally
      4. public demo 7-DoF arm
    """
    if urdf is not None:
        path = Path(urdf).expanduser().resolve()
        if not path.exists():
            raise FileNotFoundError(path)
        return path
    env = os.environ.get("PARAM_ID_URDF")
    if env:
        path = Path(env).expanduser().resolve()
        if not path.exists():
            raise FileNotFoundError(f"PARAM_ID_URDF not found: {path}")
        return path
    if PROPRIETARY_LEFT_URDF.exists():
        return PROPRIETARY_LEFT_URDF.resolve()
    return DEMO_URDF.resolve()


def package_dirs_for(urdf_path: Path) -> list[str]:
    """Pinocchio mesh search paths (repo root + proprietary assets)."""
    dirs = [str(ROOT), str(ROOT / "assets" / "proprietary")]
    # also allow meshes next to urdf parent trees
    dirs.append(str(urdf_path.parent.parent))
    return dirs


def build_model(
    urdf_path: str | Path | None = None,
) -> tuple[pin.Model, pin.Data, list[str], Path]:
    """Build Pinocchio model; returns (model, data, joint_names, urdf_path)."""
    path = resolve_urdf(urdf_path)
    model = pin.buildModelFromUrdf(str(path))
    data = model.createData()
    joint_names = [model.names[i] for i in range(1, model.njoints)]
    return model, data, joint_names, path


# Backwards-compatible alias used by older scripts
def build_left_arm_model(
    urdf_path: str | Path | None = None,
    keep_base: bool = False,
) -> tuple[pin.Model, pin.Data, list[str]]:
    if keep_base:
        raise NotImplementedError("keep_base is not supported in this release")
    model, data, names, _ = build_model(urdf_path)
    return model, data, names


def joint_limits(model: pin.Model) -> tuple[np.ndarray, np.ndarray]:
    q_min = model.lowerPositionLimit.copy()
    q_max = model.upperPositionLimit.copy()
    for i in range(model.nq):
        if not np.isfinite(q_min[i]):
            q_min[i] = -np.pi
        if not np.isfinite(q_max[i]):
            q_max[i] = np.pi
    span = q_max - q_min
    too_wide = span > 2 * np.pi + 1e-6
    mid = 0.5 * (q_min + q_max)
    q_min = np.where(too_wide, mid - np.pi * 0.8, q_min)
    q_max = np.where(too_wide, mid + np.pi * 0.8, q_max)
    tiny = (q_max - q_min) < 1e-3
    q_min = np.where(tiny, -0.5, q_min)
    q_max = np.where(tiny, 0.5, q_max)
    return q_min, q_max


def extract_inertial_params(model: pin.Model) -> np.ndarray:
    """Stack body inertial parameters in Pinocchio regressor order (10 per body)."""
    params = []
    for i in range(1, model.njoints):
        params.extend(model.inertias[i].toDynamicParameters())
    return np.asarray(params, dtype=float)


if __name__ == "__main__":
    model, data, names, path = build_model()
    print(f"urdf={path}")
    print(f"nq={model.nq}, nv={model.nv}")
    print("joints:", names)
