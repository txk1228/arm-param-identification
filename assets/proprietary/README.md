# Proprietary / private robot assets (local only)

Place employer or confidential URDF/meshes **here**. This directory is **gitignored**
except for this README.

## Suggested layout

```text
assets/proprietary/
  urdf/
    i7_left_arm.urdf
    i7.urdf
  meshes/
    *.STL
  joint_ctrl_config_new.json
```

## Use with this repo

```bash
# one-shot
python scripts/identify_static.py --urdf assets/proprietary/urdf/i7_left_arm.urdf

# or export (scripts auto-pick proprietary left arm if present)
export PARAM_ID_URDF=$PWD/assets/proprietary/urdf/i7_left_arm.urdf
```

Mesh paths inside the URDF should resolve via `package_dirs` = repo root and
`assets/proprietary` (see `param_id.robot_model.package_dirs_for`).

## Legal

Do **not** push proprietary CAD to a public GitHub repository without written
authorization from the IP owner.
