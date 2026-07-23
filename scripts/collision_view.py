#!/usr/bin/env python3
"""Excitation trajectory playback + collision check.

对齐 PDF「Excitation Trajectory — Collision View」：
  - 碰撞：Pinocchio + hppfcl（读 URDF collision mesh）
  - 可视化：Trimesh SceneViewer（可自行录屏）

你自己操作：

  conda activate env_isaaclab
  cd ~/txk/param_id

  # A. 只做碰撞检查（无窗口，先跑这个）
  python scripts/collision_view.py --traj fourier --check-only
  python scripts/collision_view.py --traj cosine --check-only
  python scripts/collision_view.py --traj cv --check-only

  # B. 打开 3D 窗口回放（录屏用这个）
  python scripts/collision_view.py --traj fourier --view
  # 窗口快捷键: SPACE 播放/暂停 | ←/→ 单帧 | Q 退出
"""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pinocchio as pin
import trimesh

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))

from param_id.robot_model import build_model, joint_limits, package_dirs_for
from param_id.trajectory import cosine_static_trajectory, fourier_trajectory

ROOT = _ROOT


# ---------------------------------------------------------------------------
# Geometry / collision (Pinocchio + hppfcl)
# ---------------------------------------------------------------------------


def build_collision_model(model: pin.Model, urdf_path: Path) -> pin.GeometryModel:
    geom = pin.buildGeomFromUrdf(
        model,
        str(urdf_path),
        pin.GeometryType.COLLISION,
        package_dirs=package_dirs_for(urdf_path),
    )
    geom.addAllCollisionPairs()
    # 去掉同一关节挂载、以及父子连杆的碰撞对（关节处本就接触）
    remove = []
    for k, pair in enumerate(geom.collisionPairs):
        i, j = pair.first, pair.second
        ji = geom.geometryObjects[i].parentJoint
        jj = geom.geometryObjects[j].parentJoint
        if ji == jj or model.parents[ji] == jj or model.parents[jj] == ji:
            remove.append(k)
    for k in reversed(remove):
        geom.removeCollisionPair(geom.collisionPairs[k])
    return geom


def colliding_pairs(
    model: pin.Model,
    data: pin.Data,
    geom: pin.GeometryModel,
    gdata: pin.GeometryData,
    q: np.ndarray,
    ignore_base: bool = False,
) -> list[tuple[str, str]]:
    pin.computeCollisions(model, data, geom, gdata, q, False)
    hits = []
    for k, pair in enumerate(geom.collisionPairs):
        if not gdata.collisionResults[k].isCollision():
            continue
        a = geom.geometryObjects[pair.first].name
        b = geom.geometryObjects[pair.second].name
        if ignore_base and ("base_link" in a or "base_link" in b):
            continue
        hits.append((a, b))
    return hits


# ---------------------------------------------------------------------------
# Trimesh visuals
# ---------------------------------------------------------------------------


def _resolve_mesh(filename: str, urdf_path: Path) -> Path:
    p = Path(filename)
    for cand in (
        p,
        urdf_path.parent / p,
        ROOT / p,
        ROOT / "meshes" / p.name,
    ):
        if cand.exists():
            return cand.resolve()
    raise FileNotFoundError(filename)


def load_visual_meshes(urdf_path: Path) -> dict[str, trimesh.Trimesh]:
    """Load visual geometry per link (STL mesh or box/cylinder primitives)."""
    root = ET.parse(urdf_path).getroot()
    out: dict[str, trimesh.Trimesh] = {}
    for link in root.findall("link"):
        name = link.get("name")
        node = link.find("visual")
        if node is None:
            node = link.find("collision")
        if node is None:
            continue
        geom = node.find("geometry")
        if geom is None:
            continue
        mesh = None
        mesh_node = geom.find("mesh")
        box_node = geom.find("box")
        cyl_node = geom.find("cylinder")
        if mesh_node is not None and mesh_node.get("filename"):
            path = _resolve_mesh(mesh_node.get("filename"), urdf_path)
            mesh = trimesh.load(str(path), force="mesh")
            if isinstance(mesh, trimesh.Scene):
                mesh = trimesh.util.concatenate(tuple(mesh.geometry.values()))
        elif box_node is not None:
            size = [float(x) for x in box_node.get("size").split()]
            mesh = trimesh.creation.box(extents=size)
        elif cyl_node is not None:
            radius = float(cyl_node.get("radius"))
            length = float(cyl_node.get("length"))
            mesh = trimesh.creation.cylinder(radius=radius, height=length)
        if mesh is None:
            continue
        origin = node.find("origin")
        if origin is not None:
            xyz = [float(x) for x in (origin.get("xyz") or "0 0 0").split()]
            rpy = [float(x) for x in (origin.get("rpy") or "0 0 0").split()]
            T = np.eye(4)
            T[:3, :3] = pin.rpy.rpyToMatrix(*rpy)
            T[:3, 3] = xyz
            mesh.apply_transform(T)
        mesh.visual.face_colors = [170, 170, 170, 255]
        out[name] = mesh
    return out


def link_world_poses(model: pin.Model, data: pin.Data) -> dict[str, np.ndarray]:
    poses = {}
    for i in range(model.njoints):
        poses[model.names[i]] = np.asarray(data.oMi[i].homogeneous)
    for fid in range(model.nframes):
        poses[model.frames[fid].name] = np.asarray(data.oMf[fid].homogeneous)
    # base_link often equals universe / joint 0
    poses.setdefault("base_link", np.asarray(data.oMi[0].homogeneous))
    poses.setdefault("universe", np.asarray(data.oMi[0].homogeneous))
    return poses


# ---------------------------------------------------------------------------
# Trajectories
# ---------------------------------------------------------------------------


def make_trajectory(args, q_min, q_max, rng):
    if args.traj == "cosine":
        t, q, _ = cosine_static_trajectory(
            q_min, q_max, fs=args.fs, duration=args.duration
        )
        return t, q
    n_periods = args.n_periods
    seed = args.seed
    if args.traj == "cv":
        n_periods = max(1, args.n_periods // 2)
        seed = args.seed + 99
    t, q, _, _ = fourier_trajectory(
        q_min,
        q_max,
        fs=args.fs,
        fundamental_freq=args.fundamental_freq,
        n_periods=n_periods,
        harmonics=args.harmonics,
        amplitude_scale=args.amplitude_scale,
        rng=np.random.default_rng(seed),
    )
    return t, q


# ---------------------------------------------------------------------------
# Check / View
# ---------------------------------------------------------------------------


def run_check(model, data, geom, gdata, t, q, subsample, ignore_base) -> dict:
    hits_frames = []
    pair_set = set()
    idxs = list(range(0, len(t), max(1, subsample)))
    for i in idxs:
        pairs = colliding_pairs(model, data, geom, gdata, q[i], ignore_base=ignore_base)
        if pairs:
            hits_frames.append(i)
            pair_set.update(pairs)
    return {
        "n_frames": len(idxs),
        "n_hit": len(hits_frames),
        "hit_frames": hits_frames[:30],
        "hit_pairs": sorted(pair_set),
        "ok": len(hits_frames) == 0,
    }


def run_viewer(model, data, geom, gdata, visuals, t, q, ignore_base) -> None:
    """Continuous playback via SceneViewer.callback (forces ~20Hz redraw)."""
    from pyglet.window import key
    from trimesh.viewer.windowed import SceneViewer

    pin.forwardKinematics(model, data, q[0])
    pin.updateFramePlacements(model, data)
    poses = link_world_poses(model, data)

    scene = trimesh.Scene()
    used = []
    for name, mesh in visuals.items():
        if name not in poses:
            continue
        scene.add_geometry(mesh.copy(), geom_name=name, transform=poses[name])
        used.append(name)

    state = {"i": 0, "playing": True}

    def apply_frame(scene_obj: trimesh.Scene) -> None:
        i = state["i"]
        pin.forwardKinematics(model, data, q[i])
        pin.updateFramePlacements(model, data)
        poses_i = link_world_poses(model, data)
        for name in used:
            if name in poses_i:
                scene_obj.graph.update(frame_to=name, matrix=poses_i[name])
        pairs = colliding_pairs(
            model, data, geom, gdata, q[i], ignore_base=ignore_base
        )
        status = "COLLISION: RED" if pairs else "COLLISION: green"
        extra = f" {pairs[0]}" if pairs else ""
        mode = "PLAY" if state["playing"] else "PAUSE"
        print(f"[{t[i]:6.2f}s / {t[-1]:.1f}s] {status} | {mode}{extra}   ", end="\r")

    def animation_callback(scene_obj: trimesh.Scene) -> None:
        # SceneViewer 在每次 on_draw 前调用；配合 callback_period 才会连续刷
        if state["playing"]:
            state["i"] = (state["i"] + 1) % len(t)
        apply_frame(scene_obj)

    class CollisionViewer(SceneViewer):
        def on_key_press(self, symbol, modifiers):
            if symbol == key.SPACE:
                state["playing"] = not state["playing"]
                print(f"\nplaying={state['playing']}")
                return
            # 方向键留给相机；单帧用 N/P
            if symbol in (key.N, key.BRACKETRIGHT, key.PERIOD):
                state["playing"] = False
                state["i"] = (state["i"] + 1) % len(t)
                apply_frame(self.scene)
                return
            if symbol in (key.B, key.BRACKETLEFT, key.COMMA):
                state["playing"] = False
                state["i"] = (state["i"] - 1) % len(t)
                apply_frame(self.scene)
                return
            if symbol == key.ESCAPE:
                self.close()
                return
            return super().on_key_press(symbol, modifiers)

    print("=" * 60)
    print("Excitation Trajectory — Collision View")
    print("  打开后应自动连续播放（标题/终端显示 PLAY）")
    print("  SPACE   = 暂停 / 继续")
    print("  N / .   = 下一帧（会先暂停）")
    print("  B / ,   = 上一帧（会先暂停）")
    print("  方向键 = 旋转相机")
    print("  ESC / Q = 退出")
    print("=" * 60)

    CollisionViewer(
        scene=scene,
        smooth=False,
        callback=animation_callback,
        callback_period=1.0 / 20.0,
        start_loop=True,
    )



def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--traj", choices=["fourier", "cosine", "cv"], default="fourier")
    p.add_argument("--check-only", action="store_true")
    p.add_argument("--view", action="store_true")
    p.add_argument("--fs", type=float, default=50.0)
    p.add_argument("--fundamental-freq", type=float, default=0.1)
    p.add_argument("--n-periods", type=int, default=2)
    p.add_argument("--harmonics", type=int, default=5)
    p.add_argument("--amplitude-scale", type=float, default=0.55)
    p.add_argument("--duration", type=float, default=20.0)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--subsample", type=int, default=2)
    p.add_argument(
        "--ignore-base",
        action="store_true",
        help="忽略与 base_link 的碰撞（只看手臂自碰，调试用）",
    )
    p.add_argument("--urdf", type=str, default=None, help="URDF path (default: auto)")
    args = p.parse_args()
    if not args.view:
        args.check_only = True

    model, data, names, urdf_path = build_model(args.urdf)
    q_min, q_max = joint_limits(model)
    print(f"[collision_view] urdf={urdf_path.name}")
    print(f"[collision_view] joints={names}")
    print(f"[collision_view] loading collision geometry...")
    geom = build_collision_model(model, urdf_path)
    gdata = pin.GeometryData(geom)
    print(f"[collision_view] geoms={geom.ngeoms}, pairs={len(geom.collisionPairs)}")

    t, q = make_trajectory(args, q_min, q_max, np.random.default_rng(args.seed))
    print(f"[collision_view] traj={args.traj} frames={len(t)} T={t[-1]:.1f}s")

    report = run_check(
        model, data, geom, gdata, t, q, args.subsample, args.ignore_base
    )
    print("-" * 50)
    print(f"checked frames : {report['n_frames']}")
    print(f"collision hits : {report['n_hit']}")
    if report["ok"]:
        print("result         : OK — COLLISION green（无非邻接碰撞）")
    else:
        print("result         : HIT — COLLISION RED")
        print("pairs          :", report["hit_pairs"][:8])
        print("example frames :", report["hit_frames"][:8])
        print("提示: 减小 --amplitude-scale，或加 --ignore-base 只查手臂自碰")
    print("-" * 50)

    out = ROOT / "results" / f"collision_{args.traj}.txt"
    out.parent.mkdir(exist_ok=True)
    out.write_text(
        f"traj={args.traj}\nok={report['ok']}\nn_hit={report['n_hit']}\n"
        f"pairs={report['hit_pairs']}\n"
    )
    print(f"saved -> {out}")

    if args.view:
        visuals = load_visual_meshes(urdf_path)
        print(f"[collision_view] visual meshes={len(visuals)}")
        try:
            run_viewer(
                model, data, geom, gdata, visuals, t, q, ignore_base=args.ignore_base
            )
        except Exception as e:
            print(f"[view] 打开窗口失败: {e}")
            print("无桌面/OpenGL 时只用 --check-only")


if __name__ == "__main__":
    main()
