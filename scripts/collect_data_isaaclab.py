#!/usr/bin/env python3
"""Isaac Lab data collection for parameter identification.

Control modes
-------------
- ``position_servo`` (default): Isaac implicit joint position drive.
- ``pd_torque``: explicit ``tau = Kp(q_des-q) + Kd(dq_des-dq)`` via effort targets.

Engineering switches (all optional, default off / legacy behaviour)
------------------------------------------------------------------
- ``--ideal-physics``: zero joint friction (baseline alignment).
- ``--enable-friction``: set PhysX static / Coulomb / viscous friction.
- ``--noise-std`` / ``--q-noise-std`` / ``--tau-noise-std``: sensor noise on logs.

Usage (``env_isaaclab``)::

    # legacy / baseline
    python scripts/collect_data_isaaclab.py --mode dynamic --traj fourier \\
        --n-periods 1 --ddq-mode ideal --ideal-physics --headless

    # engineering: PD torque + friction + noise
    python scripts/collect_data_isaaclab.py --mode dynamic --traj fourier \\
        --control-mode pd_torque --kp 200 --kd 20 \\
        --enable-friction --noise-std 0.0 --q-noise-std 1e-4 --tau-noise-std 0.05 \\
        --n-periods 1 --headless
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Pinocchio MUST be imported before AppLauncher. Isaac Sim ships its own
# (incompatible) pinocchio/eigenpy bindings; loading Kit first causes
# TypeError on std::vector<std::string> when reading model.names, etc.
# ---------------------------------------------------------------------------
_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

if sys.platform != "win32":
    import pinocchio  # noqa: F401

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser(
    description="Collect (q,dq,ddq,tau) from Isaac Lab for param_id."
)
parser.add_argument(
    "--mode",
    choices=["static", "dynamic"],
    default="dynamic",
    help="Collection mode label (stored in traj_type metadata).",
)
parser.add_argument(
    "--traj",
    choices=["fourier", "cosine"],
    default="fourier",
    help="Excitation trajectory type.",
)
parser.add_argument("--n-periods", type=int, default=10, help="Trajectory periods/cycles.")
parser.add_argument(
    "--fourier-harmonics",
    type=int,
    default=5,
    help="Fourier harmonics (fourier traj only).",
)
parser.add_argument(
    "--save-path",
    type=str,
    default=None,
    help="Output NPZ path (default under results/baseline/).",
)
parser.add_argument(
    "--dt",
    type=float,
    default=0.01,
    help="Physics / sample step [s] (default 0.01 = 100 Hz).",
)
parser.add_argument(
    "--urdf",
    type=str,
    default=None,
    help="Optional URDF path (default: param_id.robot_model.resolve_urdf).",
)
parser.add_argument(
    "--warmup-steps",
    type=int,
    default=50,
    help="Settle at first waypoint before recording.",
)
parser.add_argument(
    "--ddq-mode",
    choices=["ideal", "measured"],
    default="ideal",
    help=(
        "How to obtain acceleration: 'ideal' uses trajectory ddq_des; "
        "'measured' uses central difference on logged dq + moving-average filter."
    ),
)
parser.add_argument(
    "--ddq-ma-window",
    type=int,
    default=5,
    help="Odd window length for measured-mode moving-average filter (default 5).",
)
parser.add_argument("--seed", type=int, default=0, help="Fourier / noise RNG seed.")
parser.add_argument(
    "--fundamental-freq",
    type=float,
    default=0.1,
    help="Fourier fundamental frequency [Hz] (also maps cosine duration).",
)
parser.add_argument(
    "--ideal-physics",
    action="store_true",
    help=(
        "Baseline alignment: keep gravity, zero joint friction/viscous/armature; "
        "stiff position servo when control-mode=position_servo."
    ),
)
# ---- Stage 5: engineering switches ----
parser.add_argument(
    "--control-mode",
    choices=["position_servo", "pd_torque"],
    default="position_servo",
    help="position_servo: implicit PhysX PD; pd_torque: explicit effort PD law.",
)
parser.add_argument(
    "--kp",
    type=float,
    default=200.0,
    help="PD proportional gain [N·m/rad] for --control-mode pd_torque (default 200).",
)
parser.add_argument(
    "--kd",
    type=float,
    default=20.0,
    help="PD derivative gain [N·m·s/rad] for --control-mode pd_torque (default 20).",
)
parser.add_argument(
    "--enable-friction",
    action="store_true",
    help="Enable PhysX joint static / Coulomb / viscous friction (ignored if --ideal-physics).",
)
parser.add_argument(
    "--friction-static",
    type=float,
    default=0.5,
    help="PhysX static friction coefficient μ_s (default 0.5).",
)
parser.add_argument(
    "--friction-coulomb",
    type=float,
    default=0.3,
    help="PhysX dynamic/Coulomb friction coefficient μ_d (default 0.3).",
)
parser.add_argument(
    "--friction-viscous",
    type=float,
    default=0.05,
    help="PhysX viscous friction c_v [N·m·s/rad] (default 0.05).",
)
parser.add_argument(
    "--noise-std",
    type=float,
    default=0.0,
    help="Master Gaussian noise std; used for q and tau unless overridden (0=off).",
)
parser.add_argument(
    "--q-noise-std",
    type=float,
    default=None,
    help="Position noise std [rad] (default: --noise-std).",
)
parser.add_argument(
    "--tau-noise-std",
    type=float,
    default=None,
    help="Torque noise std [N·m] (default: --noise-std).",
)
# AppLauncher provides --headless, --device, etc.
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ---------------------------------------------------------------------------
# Imports that require a running SimulationApp
# ---------------------------------------------------------------------------
import numpy as np  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import IdealPDActuatorCfg, ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg, AssetBaseCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402

from param_id.robot_model import build_model, joint_limits, resolve_urdf  # noqa: E402
from utils.collect_extras import inject_gaussian_noise, joint_pd_torque  # noqa: E402
from utils.data_io import save_dataset  # noqa: E402
from utils.ddq import compute_ddq  # noqa: E402
from utils.traj_generator import generate_excitation  # noqa: E402


def _build_robot_cfg(
    urdf_path: Path,
    *,
    ideal_physics: bool = False,
    control_mode: str = "position_servo",
    enable_friction: bool = False,
    friction_static: float = 0.5,
    friction_coulomb: float = 0.3,
    friction_viscous: float = 0.05,
) -> ArticulationCfg:
    """Build arm ArticulationCfg for the selected control / friction setup."""
    # Actuator model
    if control_mode == "pd_torque":
        # Explicit effort mode: PhysX joint drive gains off; we send tau ourselves.
        # Friction coeffs can still be set on the actuator / via write_*_to_sim.
        fric_kw: dict = {}
        if ideal_physics:
            fric_kw = dict(
                armature=0.0,
                friction=0.0,
                dynamic_friction=0.0,
                viscous_friction=0.0,
            )
        elif enable_friction:
            fric_kw = dict(
                friction=friction_static,
                dynamic_friction=friction_coulomb,
                viscous_friction=friction_viscous,
            )
        act = IdealPDActuatorCfg(
            joint_names_expr=[".*"],
            stiffness=0.0,
            damping=0.0,
            effort_limit=500.0,
            effort_limit_sim=500.0,
            velocity_limit_sim=20.0,
            **fric_kw,
        )
    elif ideal_physics:
        act = ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            stiffness=8000.0,
            damping=80.0,
            armature=0.0,
            friction=0.0,
            dynamic_friction=0.0,
            viscous_friction=0.0,
            effort_limit_sim=500.0,
            velocity_limit_sim=20.0,
        )
    else:
        fric_kw = {}
        if enable_friction:
            fric_kw = dict(
                friction=friction_static,
                dynamic_friction=friction_coulomb,
                viscous_friction=friction_viscous,
            )
        act = ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            stiffness=400.0,
            damping=40.0,
            effort_limit_sim=200.0,
            velocity_limit_sim=10.0,
            **fric_kw,
        )

    return ArticulationCfg(
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
                    stiffness=None,
                    damping=None,
                )
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            joint_pos={".*": 0.0},
        ),
        actuators={"arm": act},
    )


@configclass
class CollectSceneCfg(InteractiveSceneCfg):
    """Minimal scene: ground plane + dome light + one arm."""

    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
    )
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )
    robot: ArticulationCfg = None  # type: ignore[assignment]


def _to_torch_row(x: np.ndarray, device: str) -> torch.Tensor:
    """(n_joint,) -> (1, n_joint) float tensor on sim device."""
    return torch.tensor(x, dtype=torch.float32, device=device).unsqueeze(0)


def _apply_friction_to_sim(
    robot: Articulation,
    *,
    static: float,
    coulomb: float,
    viscous: float,
) -> None:
    """Push friction coefficients into PhysX after reset."""
    robot.write_joint_friction_coefficient_to_sim(
        joint_friction_coeff=float(static),
        joint_dynamic_friction_coeff=float(coulomb),
        joint_viscous_friction_coeff=float(viscous),
    )


def _apply_control(
    robot: Articulation,
    *,
    control_mode: str,
    q_des_t: torch.Tensor,
    dq_des_t: torch.Tensor,
    kp: float,
    kd: float,
) -> torch.Tensor:
    """Apply one control step; return commanded / applied torque row (1, nv)."""
    if control_mode == "position_servo":
        robot.set_joint_position_target(q_des_t)
        robot.set_joint_velocity_target(dq_des_t)
        # applied_torque filled after write_data_to_sim / actuator model
        return robot.data.applied_torque
    # Explicit PD torque
    q = robot.data.joint_pos
    dq = robot.data.joint_vel
    tau = kp * (q_des_t - q) + kd * (dq_des_t - dq)
    robot.set_joint_effort_target(tau)
    return tau


def run_collection(
    sim: SimulationContext,
    scene: InteractiveScene,
    q_des: np.ndarray,
    dq_des: np.ndarray,
    warmup_steps: int,
    *,
    control_mode: str,
    kp: float,
    kd: float,
    apply_friction: bool,
    friction_static: float,
    friction_coulomb: float,
    friction_viscous: float,
) -> dict[str, np.ndarray]:
    """Track desired trajectory; return measured q, dq, tau (ddq filled later)."""
    robot: Articulation = scene["robot"]
    device = sim.device
    sim_dt = sim.get_physics_dt()
    n_steps, n_joint = q_des.shape

    root_state = robot.data.default_root_state.clone()
    root_state[:, :3] += scene.env_origins
    robot.write_root_pose_to_sim(root_state[:, :7])
    robot.write_root_velocity_to_sim(root_state[:, 7:])
    q0 = _to_torch_row(q_des[0], device)
    dq0 = torch.zeros_like(q0)
    robot.write_joint_state_to_sim(q0, dq0)
    robot.reset()
    scene.reset()

    if apply_friction:
        _apply_friction_to_sim(
            robot,
            static=friction_static,
            coulomb=friction_coulomb,
            viscous=friction_viscous,
        )
        print(
            f"[collect] friction ON  μ_s={friction_static}  "
            f"μ_d={friction_coulomb}  c_v={friction_viscous}"
        )

    for _ in range(max(0, warmup_steps)):
        _apply_control(
            robot,
            control_mode=control_mode,
            q_des_t=q0,
            dq_des_t=dq0,
            kp=kp,
            kd=kd,
        )
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

    q_log = np.zeros((n_steps, n_joint), dtype=np.float64)
    dq_log = np.zeros_like(q_log)
    tau_log = np.zeros_like(q_log)

    print(
        f"[collect] recording {n_steps} steps @ dt={sim_dt:.4f} s "
        f"control={control_mode} ..."
    )
    for i in range(n_steps):
        if not simulation_app.is_running():
            raise RuntimeError("SimulationApp stopped early")

        qi = _to_torch_row(q_des[i], device)
        dqi = _to_torch_row(dq_des[i], device)
        tau_cmd = _apply_control(
            robot,
            control_mode=control_mode,
            q_des_t=qi,
            dq_des_t=dqi,
            kp=kp,
            kd=kd,
        )
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

        q_log[i] = robot.data.joint_pos[0].detach().cpu().numpy()
        dq_log[i] = robot.data.joint_vel[0].detach().cpu().numpy()
        if control_mode == "pd_torque":
            # Commanded PD torque (what we sent); also matches IdealPD applied_effort.
            tau_log[i] = tau_cmd[0].detach().cpu().numpy()
        else:
            # Implicit drive: use actuator applied_torque (not effort target).
            tau_log[i] = robot.data.applied_torque[0].detach().cpu().numpy()

        if (i + 1) % max(1, n_steps // 10) == 0 or i == n_steps - 1:
            print(f"[collect]  {i + 1}/{n_steps}")

    return {"q": q_log, "dq": dq_log, "tau": tau_log}


def main() -> None:
    urdf_path = resolve_urdf(args_cli.urdf)
    model, _, joint_names, _ = build_model(urdf_path)
    q_min, q_max = joint_limits(model)
    rng = np.random.default_rng(args_cli.seed)

    q_noise_std = (
        args_cli.noise_std if args_cli.q_noise_std is None else args_cli.q_noise_std
    )
    tau_noise_std = (
        args_cli.noise_std if args_cli.tau_noise_std is None else args_cli.tau_noise_std
    )

    apply_friction = bool(args_cli.enable_friction) and not bool(args_cli.ideal_physics)
    if args_cli.enable_friction and args_cli.ideal_physics:
        print("[collect] WARN: --ideal-physics overrides --enable-friction (friction=0)")

    t, q_des, dq_des, ddq_des = generate_excitation(
        args_cli.traj,
        q_min,
        q_max,
        dt=args_cli.dt,
        n_periods=args_cli.n_periods,
        fourier_harmonics=args_cli.fourier_harmonics,
        fundamental_freq=args_cli.fundamental_freq,
        seed=args_cli.seed,
    )
    _ = t

    save_path = args_cli.save_path
    if save_path is None:
        tag = "ideal" if args_cli.ideal_physics else args_cli.ddq_mode
        save_path = str(
            _ROOT
            / "results"
            / "baseline"
            / f"isaac_{args_cli.mode}_{args_cli.traj}_{tag}.npz"
        )

    print(f"[collect] urdf={urdf_path}")
    print(f"[collect] joints({model.nv})={joint_names}")
    print(f"[collect] mode={args_cli.mode} traj={args_cli.traj} N={q_des.shape[0]}")
    print(f"[collect] control_mode={args_cli.control_mode}  kp={args_cli.kp} kd={args_cli.kd}")
    print(f"[collect] ddq_mode={args_cli.ddq_mode} ma_window={args_cli.ddq_ma_window}")
    print(f"[collect] ideal_physics={args_cli.ideal_physics} enable_friction={apply_friction}")
    print(f"[collect] q_noise_std={q_noise_std} tau_noise_std={tau_noise_std}")
    print(f"[collect] headless={getattr(args_cli, 'headless', False)}")
    print(f"[collect] save_path={save_path}")

    sim_cfg = sim_utils.SimulationCfg(dt=args_cli.dt, device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([2.0, 2.0, 1.5], [0.0, 0.0, 0.6])

    scene_cfg = CollectSceneCfg(num_envs=1, env_spacing=2.0)
    scene_cfg.robot = _build_robot_cfg(
        urdf_path,
        ideal_physics=args_cli.ideal_physics,
        control_mode=args_cli.control_mode,
        enable_friction=apply_friction,
        friction_static=args_cli.friction_static,
        friction_coulomb=args_cli.friction_coulomb,
        friction_viscous=args_cli.friction_viscous,
    )
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    print("[collect] setup complete")

    logs = run_collection(
        sim,
        scene,
        q_des=q_des,
        dq_des=dq_des,
        warmup_steps=args_cli.warmup_steps,
        control_mode=args_cli.control_mode,
        kp=args_cli.kp,
        kd=args_cli.kd,
        apply_friction=apply_friction,
        friction_static=args_cli.friction_static,
        friction_coulomb=args_cli.friction_coulomb,
        friction_viscous=args_cli.friction_viscous,
    )

    # Sensor noise on logged q / tau (dq left clean for differentiation option).
    q_clean = logs["q"]
    tau_clean = logs["tau"]
    q_out = inject_gaussian_noise(q_clean, q_noise_std, rng)
    tau_out = inject_gaussian_noise(tau_clean, tau_noise_std, rng)

    ddq = compute_ddq(
        args_cli.ddq_mode,
        ddq_des=ddq_des,
        dq_meas=logs["dq"],
        dt=float(args_cli.dt),
        ma_window=args_cli.ddq_ma_window,
    )
    if not np.isfinite(ddq).all():
        raise RuntimeError("ddq contains NaN/Inf after compute_ddq")

    phys = "ideal" if args_cli.ideal_physics else ("fric" if apply_friction else "default")
    traj_type = (
        f"isaac_{args_cli.mode}_{args_cli.traj}_{args_cli.ddq_mode}_"
        f"{args_cli.control_mode}_{phys}"
    )
    out = save_dataset(
        save_path,
        {
            "q": q_out,
            "dq": logs["dq"],
            "ddq": ddq,
            "tau": tau_out,
            "dt": float(args_cli.dt),
            "traj_type": traj_type,
        },
    )
    track_err = float(np.mean(np.abs(q_clean - q_des)))
    ddq_jerk_proxy = float(np.mean(np.abs(np.diff(ddq, axis=0))))
    # Sanity: PD residual on last sample (offline formula check).
    tau_pd_check = joint_pd_torque(
        q_clean[-1], logs["dq"][-1], q_des[-1], dq_des[-1], args_cli.kp, args_cli.kd
    )
    print(f"[collect] saved -> {out}")
    print(
        "[collect] tau RMS ="
        f" {np.sqrt(np.mean(tau_out ** 2)):.4f} Nm,"
        f" |q-q_des| mean = {track_err:.4e} rad,"
        f" mean|Δddq| = {ddq_jerk_proxy:.4e}"
    )
    if args_cli.control_mode == "pd_torque":
        print(
            f"[collect] PD check |tau_cmd - PD(q)| last ="
            f" {np.linalg.norm(tau_clean[-1] - tau_pd_check):.3e} Nm"
        )
    if q_noise_std > 0 or tau_noise_std > 0:
        print(
            f"[collect] noise Δq RMS={np.sqrt(np.mean((q_out - q_clean) ** 2)):.4e}, "
            f"Δtau RMS={np.sqrt(np.mean((tau_out - tau_clean) ** 2)):.4e}"
        )


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        # Print before close(): SimulationApp.close() often aborts the process
        # without flushing a normal Python traceback.
        import traceback

        traceback.print_exc()
        raise
    finally:
        simulation_app.close()
