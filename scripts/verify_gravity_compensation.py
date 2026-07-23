#!/usr/bin/env python3
"""Stage 7: gravity-compensation closed-loop check in Isaac Lab.

Loads static identification results, applies real-time gravity feedforward
``tau = Kp(q*-q) + Kd(dq*-dq) + tau_g(q)``, compares residual effort with/without
compensation, and runs a light end-effector wrench “zero-force drag” demo.

Also supports ``--offline`` (Pinocchio-only metrics) when Isaac is unavailable.

Usage::

    conda activate env_isaaclab

    # Offline metrics (no GUI / no Isaac physics)
    python scripts/verify_gravity_compensation.py --offline \\
        --id-result results/baseline/id_pinocchio_static/static_ols.npz

    # Isaac Lab hold + drag
    python scripts/verify_gravity_compensation.py --headless \\
        --id-result results/baseline/id_pinocchio_static/static_ols.npz \\
        --out-dir results/gravity_comp
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

parser = argparse.ArgumentParser(description="Gravity compensation verification")
parser.add_argument(
    "--id-result",
    type=str,
    default=str(_ROOT / "results" / "baseline" / "id_pinocchio_static" / "static_ols.npz"),
    help="static_*.npz from identify_static.py",
)
parser.add_argument("--urdf", type=str, default=None)
parser.add_argument(
    "--offline",
    action="store_true",
    help="Skip Isaac; evaluate residual gravity error in Pinocchio only.",
)
parser.add_argument("--dt", type=float, default=0.01)
parser.add_argument("--kp", type=float, default=80.0, help="Hold-test P gain.")
parser.add_argument("--kd", type=float, default=8.0, help="D gain (hold + drag).")
parser.add_argument("--hold-steps", type=int, default=150, help="Steps per posture.")
parser.add_argument("--drag-steps", type=int, default=300, help="Zero-force drag steps.")
parser.add_argument(
    "--ee-force",
    type=float,
    default=3.0,
    help="End-effector force magnitude [N] for drag (world +Y).",
)
parser.add_argument(
    "--out-dir",
    type=str,
    default=str(_ROOT / "results" / "gravity_comp"),
)
parser.add_argument("--seed", type=int, default=0)

# Parse early for --offline so we can skip AppLauncher.
_pre = argparse.ArgumentParser(add_help=False)
_pre.add_argument("--offline", action="store_true")
_pre_args, _ = _pre.parse_known_args()

simulation_app = None
if not _pre_args.offline:
    # Pinocchio before AppLauncher (Isaac Sim ships incompatible bindings).
    if sys.platform != "win32":
        import pinocchio  # noqa: F401
    from isaaclab.app import AppLauncher  # noqa: E402

    AppLauncher.add_app_launcher_args(parser)
    args_cli = parser.parse_args()
    app_launcher = AppLauncher(args_cli)
    simulation_app = app_launcher.app
else:
    # Allow leftover AppLauncher flags (e.g. --headless) without failing.
    args_cli, _unknown = parser.parse_known_args()
    args_cli.offline = True

import numpy as np  # noqa: E402

from param_id.robot_model import build_model, joint_limits  # noqa: E402
from utils.gravity_comp import (  # noqa: E402
    evaluate_compensation_offline,
    gravity_torque_identified,
    gravity_torque_urdf,
    load_static_id_result,
    residual_stats,
)


def _sample_postures(model, n: int = 8, seed: int = 0) -> np.ndarray:
    q_min, q_max = joint_limits(model)
    rng = np.random.default_rng(seed)
    return rng.uniform(q_min + 0.1, q_max - 0.1, size=(n, model.nv))


def run_offline(args) -> dict:
    model, data, names, urdf = build_model(args.urdf)
    idres = load_static_id_result(args.id_result)
    q_list = _sample_postures(model, n=12, seed=args.seed)
    metrics = evaluate_compensation_offline(
        model, data, q_list, idres["pi_hat"], idres["idx_g"]
    )
    out = {
        "mode": "offline",
        "urdf": str(urdf),
        "id_result": str(args.id_result),
        "joints": names,
        "metrics": metrics,
        "pass": metrics["identified"]["mean_norm"]
        < 0.05 * max(metrics["none"]["mean_norm"], 1e-6),
    }
    return out


def run_isaac(args) -> dict:
    import torch

    import isaaclab.sim as sim_utils
    from isaaclab.actuators import IdealPDActuatorCfg
    from isaaclab.assets import Articulation, ArticulationCfg, AssetBaseCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.sim import SimulationContext
    from isaaclab.utils import configclass

    from param_id.robot_model import resolve_urdf

    model, data, names, urdf = build_model(args.urdf)
    idres = load_static_id_result(args.id_result)
    pi_hat, idx_g = idres["pi_hat"], idres["idx_g"]
    urdf_path = resolve_urdf(args.urdf)

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        ground = AssetBaseCfg(
            prim_path="/World/defaultGroundPlane",
            spawn=sim_utils.GroundPlaneCfg(),
        )
        dome_light = AssetBaseCfg(
            prim_path="/World/Light",
            spawn=sim_utils.DomeLightCfg(intensity=2500.0, color=(0.75, 0.75, 0.75)),
        )
        robot = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Robot",
            spawn=sim_utils.UrdfFileCfg(
                asset_path=str(urdf_path),
                fix_base=True,
                merge_fixed_joints=True,
                make_instanceable=False,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    disable_gravity=False,
                    max_depenetration_velocity=5.0,
                ),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                    enabled_self_collisions=False,
                    solver_position_iteration_count=8,
                    solver_velocity_iteration_count=0,
                ),
                joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
                    gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                        stiffness=None, damping=None
                    )
                ),
            ),
            init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0), joint_pos={".*": 0.0}),
            actuators={
                "arm": IdealPDActuatorCfg(
                    joint_names_expr=[".*"],
                    stiffness=0.0,
                    damping=0.0,
                    armature=0.0,
                    friction=0.0,
                    dynamic_friction=0.0,
                    viscous_friction=0.0,
                    effort_limit=300.0,
                    effort_limit_sim=300.0,
                ),
            },
        )

    sim = SimulationContext(sim_utils.SimulationCfg(dt=args.dt, device=args_cli.device))
    sim.set_camera_view([2.2, 2.2, 1.4], [0.0, 0.0, 0.5])
    scene = InteractiveScene(SceneCfg(num_envs=1, env_spacing=2.0))
    sim.reset()
    robot: Articulation = scene["robot"]

    # Resolve EE body index (last link).
    body_names = list(robot.data.body_names)
    ee_candidates = [i for i, n in enumerate(body_names) if "link7" in n.lower() or n.endswith("7")]
    ee_id = ee_candidates[-1] if ee_candidates else len(body_names) - 1
    print(f"[gc] bodies={body_names}")
    print(f"[gc] EE body id={ee_id} name={body_names[ee_id]}")

    device = sim.device
    sim_dt = sim.get_physics_dt()
    nv = model.nv
    postures = _sample_postures(model, n=5, seed=args.seed)

    def _torch_q(q: np.ndarray) -> torch.Tensor:
        return torch.tensor(q, dtype=torch.float32, device=device).unsqueeze(0)

    def _reset_to(q: np.ndarray) -> None:
        root = robot.data.default_root_state.clone()
        root[:, :3] += scene.env_origins
        robot.write_root_pose_to_sim(root[:, :7])
        robot.write_root_velocity_to_sim(root[:, 7:])
        robot.write_joint_state_to_sim(_torch_q(q), torch.zeros(1, nv, device=device))
        robot.reset()
        scene.reset()
        robot.set_external_force_and_torque(
            forces=torch.zeros(0, 3, device=device),
            torques=torch.zeros(0, 3, device=device),
        )

    def _step_control(q_des: np.ndarray, dq_des: np.ndarray, tau_g: np.ndarray, use_gc: bool):
        q = robot.data.joint_pos
        dq = robot.data.joint_vel
        qd = _torch_q(q_des)
        dqd = _torch_q(dq_des)
        tau_pd = args.kp * (qd - q) + args.kd * (dqd - dq)
        tau_ff = _torch_q(tau_g) if use_gc else torch.zeros_like(tau_pd)
        tau = tau_pd + tau_ff
        robot.set_joint_effort_target(tau)
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)
        # Residual = PD effort magnitude (should drop when GC cancels gravity)
        return tau_pd[0].detach().cpu().numpy()

    def _hold_experiment(label: str, tau_g_fn) -> dict:
        """Hold postures; compare residual PD with GC off vs on."""
        res_off, res_on = [], []
        for qi in postures:
            _reset_to(qi)
            # settle without GC
            for _ in range(20):
                _step_control(qi, np.zeros(nv), np.zeros(nv), use_gc=False)
            for _ in range(args.hold_steps):
                r = _step_control(qi, np.zeros(nv), np.zeros(nv), use_gc=False)
                res_off.append(r)
            _reset_to(qi)
            for _ in range(20):
                tg = tau_g_fn(qi)
                _step_control(qi, np.zeros(nv), tg, use_gc=True)
            for _ in range(args.hold_steps):
                # Use measured q for feedforward (real-time)
                q_meas = robot.data.joint_pos[0].detach().cpu().numpy()
                tg = tau_g_fn(q_meas)
                r = _step_control(qi, np.zeros(nv), tg, use_gc=True)
                res_on.append(r)
        off = residual_stats(np.vstack(res_off))
        on = residual_stats(np.vstack(res_on))
        print(
            f"[gc][{label}] residual PD  OFF mean={off['mean_norm']:.4f} max={off['max_norm']:.4f} | "
            f"ON mean={on['mean_norm']:.4f} max={on['max_norm']:.4f}  "
            f"reduction={100*(1-on['mean_norm']/max(off['mean_norm'],1e-9)):.1f}%"
        )
        return {"off": off, "on": on}

    def _tau_urdf(q: np.ndarray) -> np.ndarray:
        return gravity_torque_urdf(model, data, q)

    def _tau_id(q: np.ndarray) -> np.ndarray:
        return gravity_torque_identified(model, data, q, pi_hat, idx_g)

    print("[gc] hold test: URDF compensation")
    hold_urdf = _hold_experiment("urdf", _tau_urdf)
    print("[gc] hold test: identified compensation")
    hold_id = _hold_experiment("identified", _tau_id)

    # ---- Zero-force drag: GC + damping only, small EE wrench ----
    print(f"[gc] drag test: EE force = {args.ee_force} N (world +Y), steps={args.drag_steps}")
    q0 = postures[0].copy()
    _reset_to(q0)
    # Apply constant force on EE in world +Y
    forces = torch.zeros(1, 1, 3, device=device)
    forces[0, 0, 1] = float(args.ee_force)
    torques = torch.zeros_like(forces)
    robot.set_external_force_and_torque(
        forces=forces,
        torques=torques,
        body_ids=[ee_id],
        is_global=True,
    )
    q_traj = []
    for _ in range(args.drag_steps):
        if simulation_app is not None and not simulation_app.is_running():
            break
        q_meas = robot.data.joint_pos[0].detach().cpu().numpy()
        tg = _tau_id(q_meas)
        # no position hold: only GC + damping
        q = robot.data.joint_pos
        dq = robot.data.joint_vel
        tau = _torch_q(tg) + args.kd * (torch.zeros_like(dq) - dq)
        robot.set_joint_effort_target(tau)
        # re-assert external wrench each step (buffers)
        robot.set_external_force_and_torque(
            forces=forces, torques=torques, body_ids=[ee_id], is_global=True
        )
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)
        q_traj.append(q_meas.copy())
    q_traj = np.asarray(q_traj)
    motion = float(np.mean(np.abs(q_traj[-1] - q_traj[0])))
    path_len = float(np.sum(np.linalg.norm(np.diff(q_traj, axis=0), axis=1)))
    print(f"[gc] drag: |Δq|_mean={motion:.4f} rad, path_length={path_len:.4f} rad")

    # Clear wrench
    robot.set_external_force_and_torque(
        forces=torch.zeros(0, 3, device=device),
        torques=torch.zeros(0, 3, device=device),
    )

    # Pass: GC residual mean << uncompensated
    ok_urdf = hold_urdf["on"]["mean_norm"] < 0.35 * hold_urdf["off"]["mean_norm"]
    ok_id = hold_id["on"]["mean_norm"] < 0.35 * hold_id["off"]["mean_norm"]
    return {
        "mode": "isaac",
        "urdf": str(urdf),
        "id_result": str(args.id_result),
        "joints": names,
        "hold_urdf": hold_urdf,
        "hold_identified": hold_id,
        "drag": {"mean_abs_delta_q": motion, "path_length": path_len, "ee_force_N": args.ee_force},
        "pass": bool(ok_urdf and ok_id and path_len > 1e-3),
    }


def _write_outputs(result: dict, out_dir: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "gravity_comp_metrics.json").write_text(
        json.dumps(result, indent=2), encoding="utf-8"
    )

    # Bar chart
    fig, ax = plt.subplots(figsize=(7.5, 4))
    if result["mode"] == "offline":
        m = result["metrics"]
        labels = ["No GC", "URDF GC", "Identified GC"]
        means = [m["none"]["mean_norm"], m["urdf"]["mean_norm"], m["identified"]["mean_norm"]]
        maxes = [m["none"]["max_norm"], m["urdf"]["max_norm"], m["identified"]["max_norm"]]
    else:
        labels = ["Hold no GC", "Hold URDF GC", "Hold ID GC"]
        means = [
            result["hold_urdf"]["off"]["mean_norm"],
            result["hold_urdf"]["on"]["mean_norm"],
            result["hold_identified"]["on"]["mean_norm"],
        ]
        maxes = [
            result["hold_urdf"]["off"]["max_norm"],
            result["hold_urdf"]["on"]["max_norm"],
            result["hold_identified"]["on"]["max_norm"],
        ]
    x = np.arange(len(labels))
    ax.bar(x - 0.15, means, 0.3, label="mean ||residual||", color="#4C72B0")
    ax.bar(x + 0.15, maxes, 0.3, label="max ||residual||", color="#DD8452")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Residual torque norm [N·m]")
    ax.set_title("Gravity compensation residual")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "fig_residual_bars.png", dpi=140)
    plt.close(fig)

    # Markdown summary
    lines = [
        "# Gravity compensation verification",
        "",
        f"- mode: `{result['mode']}`",
        f"- urdf: `{result['urdf']}`",
        f"- id_result: `{result.get('id_result')}`",
        f"- pass: **{result.get('pass')}**",
        "",
    ]
    if result["mode"] == "offline":
        m = result["metrics"]
        lines += [
            "## Offline residual vs RNEA truth",
            "",
            f"| mode | mean ||r|| | max ||r|| |",
            "|------|----------:|---------:|",
            f"| none | {m['none']['mean_norm']:.4e} | {m['none']['max_norm']:.4e} |",
            f"| urdf | {m['urdf']['mean_norm']:.4e} | {m['urdf']['max_norm']:.4e} |",
            f"| identified | {m['identified']['mean_norm']:.4e} | {m['identified']['max_norm']:.4e} |",
            "",
        ]
    else:
        lines += [
            "## Isaac hold residual (PD effort)",
            "",
            "| compensation | mean | max |",
            "|--------------|-----:|----:|",
            f"| OFF | {result['hold_urdf']['off']['mean_norm']:.4f} | {result['hold_urdf']['off']['max_norm']:.4f} |",
            f"| URDF ON | {result['hold_urdf']['on']['mean_norm']:.4f} | {result['hold_urdf']['on']['max_norm']:.4f} |",
            f"| Identified ON | {result['hold_identified']['on']['mean_norm']:.4f} | {result['hold_identified']['on']['max_norm']:.4f} |",
            "",
            f"Drag path length: {result['drag']['path_length']:.4f} rad "
            f"(EE force={result['drag']['ee_force_N']} N).",
            "",
        ]
    (out_dir / "conclusion.md").write_text("\n".join(lines), encoding="utf-8")
    print(f"[gc] wrote {out_dir / 'gravity_comp_metrics.json'}")
    print(f"[gc] wrote {out_dir / 'fig_residual_bars.png'}")
    print(f"[gc] wrote {out_dir / 'conclusion.md'}")


def main() -> None:
    out_dir = Path(args_cli.out_dir)
    if args_cli.offline:
        result = run_offline(args_cli)
    else:
        result = run_isaac(args_cli)
    _write_outputs(result, out_dir)
    print(f"[gc] PASS={result.get('pass')}")
    if not result.get("pass"):
        sys.exit(1)


if __name__ == "__main__":
    try:
        main()
    finally:
        if simulation_app is not None:
            simulation_app.close()
