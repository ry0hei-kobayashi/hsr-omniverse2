#!/usr/bin/env python3
"""Iterate over YCB objects, attempt grasps with the HSR, then unload the assets."""
from __future__ import annotations

import math
import os
import glob
from typing import Dict, Iterable, List, Tuple

from isaacsim.simulation_app import SimulationApp

# The SimulationApp must be created before importing most Isaac/Omni modules.
kit = SimulationApp({"renderer": "RayTracedLighting", "headless": False})
kit.set_setting("/app/extensions/installUntrustedExtensions", True)

import omni.kit.commands  # noqa: E402  (import after SimulationApp creation)
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import SimulationContext  # noqa: E402
from isaacsim.core.utils import stage, viewports  # noqa: E402
from isaacsim.core.utils.prims import create_prim  # noqa: E402
from isaacsim.core.utils.rotations import euler_angles_to_quat  # noqa: E402
from omni.isaac.dynamic_control import _dynamic_control  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdPhysics  # noqa: E402

from hsr_omniverse2_sim.paths import USD_DIR  # noqa: E402
from hsr_omniverse2_sim.robot.hsr import robot as hsr  # noqa: E402, needs SimulationApp

import pyassimp  # type: ignore
import numpy as np


def _resolve_models_dir() -> str:
    """tmc_wrs_gazebo_worlds の models/ (STL メッシュ) の場所。

    Isaac コンテナには ROS の tmc_wrs_gazebo_worlds パッケージは入っていない (純 Python の
    randomizer だけ) ので、Dockerfile.ycb-grasp が sparse checkout した場所を WRS_MODELS_DIR で
    受ける。ament index があればそちらも試す。無ければ空文字 → bbox は既定値になる。
    """
    env = os.environ.get('WRS_MODELS_DIR')
    if env and os.path.isdir(env):
        return env
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('tmc_wrs_gazebo_worlds'), 'models')
    except Exception:  # noqa: BLE001 - パッケージ無し
        return ''


MODELS_DIR = _resolve_models_dir()

# io/models/usd/wrs_models (submodule) の model.usd。
MODEL_ROOT = os.path.join(USD_DIR, 'wrs_models')
OBJECT_COLLECTION_PATH = "/World/YcbObjects"
SKIPPED_MODELS = {}

PREGRASP_POSE: Dict[str, float] = {
    "arm_lift_joint": 0.6,
    "arm_flex_joint": -1.62 - 0.6,
    "arm_roll_joint": 0.0,
    "wrist_flex_joint": -1.62 + 0.6,
    "wrist_roll_joint": 0.0,
    "head_pan_joint": 0.0,
    "head_tilt_joint": -0.6,
}
GRASP_POSE: Dict[str, float] = {
    "arm_lift_joint": 0.05,
    "arm_flex_joint": -1.62 - 0.6,
    "arm_roll_joint": 0.0,
    "wrist_flex_joint": -1.62 + 0.6,
    "wrist_roll_joint": 0.0,
    "head_pan_joint": 0.0,
    "head_tilt_joint": -0.6,
}
GRIPPER_OPEN = 1.0
GRIPPER_CLOSE = 0.03


def log(message: str) -> None:
    print(f"[ycb-grasp] {message}")


def find_model_mesh_stl(model_dir_name: str) -> str:
    """Try to locate an STL mesh file under the model directory.

    Prefers 'meshes/nontextured.stl', falls back to first '*.stl' under 'meshes/'.
    Returns empty string if not found.
    """
    root = os.path.join(MODELS_DIR, model_dir_name)
    candidate = os.path.join(root, 'meshes', 'nontextured.stl')
    if os.path.exists(candidate):
        return candidate
    gl = glob.glob(os.path.join(root, 'meshes', '*.stl'))
    return gl[0] if gl else ''


def parse_stl_aabb(stl_path: str) -> Tuple[float, float, float, float, float, float]:
    """Return STL axis-aligned bbox as per-axis min/max: (xmin, xmax, ymin, ymax, zmin, zmax).

    Prefer pyassimp if available; fallback to minimal manual parser.
    """
    default_aabb = (-0.05, 0.05, -0.05, 0.05, -0.05, 0.05)
    if not stl_path or not os.path.exists(stl_path):
        # Default small cube centered near zero
        return default_aabb
    try:
        xs, ys, zs = [], [], []
        with pyassimp.load(stl_path) as scene:
            for mesh in scene.meshes:
                # mesh.vertices is Nx3 float array
                for v in mesh.vertices:
                    xs.append(float(v[0]))
                    ys.append(float(v[1]))
                    zs.append(float(v[2]))
        if xs:
            return (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))
    except Exception as ex:
        log(f'pyassimp failed on {stl_path}: {ex}')
    return default_aabb


def setup_scene() -> None:
    """Set camera, lighting, and a simple background."""
    viewports.set_camera_view(
        eye=(3.0, 1.5, 3.0),
        target=(0.0, 0.0, 0.8),
    )

    # Lights
    create_prim(
        "/World/LightKey",
        "SphereLight",
        translation=(2.0, 0.0, 5.0),
        attributes={"inputs:radius": 0.01, "inputs:intensity": 4.5e4},
    )
    create_prim(
        "/World/FillLight",
        "SphereLight",
        translation=(-2.0, -1.5, 5.0),
        attributes={"inputs:radius": 0.01, "inputs:intensity": 3.0e4},
    )

    # Ground environment (launch_isaacsim.py と同じ決め方: オフラインミラー → アセットサーバ → 既知 URL)
    offline_mirror = os.path.join(USD_DIR, 'isaac_offline')
    if os.path.isdir(offline_mirror) and os.environ.get('ISAAC_FORCE_ONLINE_ASSETS') != '1':
        assets_root = offline_mirror
    else:
        assets_root = get_assets_root_path() or os.environ.get(
            'ISAAC_ASSETS_ROOT',
            'https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1',
        )
    log(f'assets root: {assets_root}')
    default_env = "/Isaac/Environments/Grid/default_environment.usd"
    stage.add_reference_to_stage(assets_root + default_env, "/World/Env")

    # Collection to keep spawned objects organized
    create_prim(OBJECT_COLLECTION_PATH, "Xform")


def create_hsr_robot() -> hsr.hsr:
    """Instantiate the HSR at a reasonable pose."""
    hsr_root = "/hsrb"
    if not omni.usd.get_context().get_stage().GetPrimAtPath(hsr_root):
        create_prim(
            prim_path=hsr_root,
            prim_type="Xform",
            translation=[0.0, 0.0, 0.0],
            orientation=euler_angles_to_quat([0.0, 0.0, 0.0]),
        )
    robot = hsr.hsr(stage_path=hsr_root)
    fix_robot_to_world(robot)
    return robot


def fix_robot_to_world(robot: hsr.hsr) -> None:
    """Attach the base link to the world using a fixed joint."""
    stage_handle = omni.usd.get_context().get_stage()
    base_link_path = robot.stage_path + robot.prefix + "/base_link"
    base_link_prim = stage_handle.GetPrimAtPath(base_link_path)
    if not base_link_prim:
        log(f"HSR base link '{base_link_path}' not found; skipping fixed joint setup.")
        return
    joint_path = robot.stage_path + robot.prefix + "/world_fixed_joint"
    joint = UsdPhysics.FixedJoint.Define(stage_handle, joint_path)
    joint.CreateBody0Rel().SetTargets(["/World"])
    joint.CreateBody1Rel().SetTargets([base_link_path])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    log("Added fixed joint to pin HSR base_link to /World.")


class HSRController:
    """Helper to send joint targets through Dynamic Control."""

    TRACKED_JOINTS = [
        "arm_lift_joint",
        "arm_flex_joint",
        "arm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint",
        "head_pan_joint",
        "head_tilt_joint",
        "hand_motor_joint",
        "torso_lift_joint",
    ]

    def __init__(self, robot: hsr.hsr) -> None:
        self._robot = robot
        self._dc = robot.dc
        self._articulation_path = robot.stage_path + robot.prefix
        self._articulation = self._acquire_articulation()
        self._dofs = self._cache_dofs(self.TRACKED_JOINTS)

    def _acquire_articulation(self):
        articulation = self._dc.get_articulation(self._articulation_path)
        if articulation == _dynamic_control.INVALID_HANDLE:
            raise RuntimeError("HSR articulation handle is invalid")
        return articulation

    def _cache_dofs(self, names: Iterable[str]):
        dofs = {}
        for name in names:
            dof = self._dc.find_articulation_dof(self._articulation, name)
            if dof == _dynamic_control.INVALID_HANDLE:
                log(f"Warning: DOF '{name}' is not available; skipping it.")
                continue
            dofs[name] = dof
        return dofs

    def set_joint_targets(self, targets: Dict[str, float]) -> None:
        self._dc.wake_up_articulation(self._articulation)
        for name, value in targets.items():
            if name == 'arm_lift_joint':
                self._dc.set_dof_position_target(self._dofs['torso_lift_joint'], value / 2.0)
                self._dc.set_dof_velocity_target(self._dofs['torso_lift_joint'], 0.0)
            dof = self._dofs.get(name)
            if dof is None:
                continue
            self._dc.set_dof_position_target(dof, value)
            self._dc.set_dof_velocity_target(dof, 0.0)

    def set_gripper(self, opening: float) -> None:
        self._robot.gripper_trajectory_action_server._set_joint_position("hand_motor_joint", opening)


def advance_simulation(sim_context: SimulationContext, robot: hsr.hsr, steps: int, render: bool = True) -> None:
    for _ in range(steps):
        sim_context.step(render=render)
        robot.step()


def list_ycb_models() -> List[str]:
    if not os.path.isdir(MODEL_ROOT):
        raise FileNotFoundError(f"Cannot locate wrs_models directory: {MODEL_ROOT}")
    names = [
        entry
        for entry in os.listdir(MODEL_ROOT)
        if entry.startswith("ycb_") and os.path.isdir(os.path.join(MODEL_ROOT, entry))
    ]
    names.sort()
    if SKIPPED_MODELS:
        names = [name for name in names if name not in SKIPPED_MODELS]
        skipped = sorted(SKIPPED_MODELS)
        for name in skipped:
            log(f"Skipping model '{name}'")
    limit = os.environ.get("YCB_MAX_MODELS")
    if limit:
        try:
            max_count = int(limit)
            names = names[:max_count]
        except ValueError:
            log(f"Invalid YCB_MAX_MODELS value '{limit}', ignoring.")
    return names


def spawn_object(model_name: str, x_offset: float, y_offset: float) -> str:
    usd_path = os.path.join(MODEL_ROOT, model_name, "model.usd")
    if not os.path.exists(usd_path):
        raise FileNotFoundError(f"Missing USD for {model_name}: {usd_path}")
    stage_path = f"{OBJECT_COLLECTION_PATH}/{model_name}"
    create_prim(stage_path, "Xform", translation=(0.39 + x_offset, 0.11 + y_offset, 0.01), orientation=euler_angles_to_quat([0.0, 0.0, math.pi/2.0]))
    stage.add_reference_to_stage(usd_path, Sdf.Path(stage_path))
    return stage_path


def delete_object(stage_path: str) -> None:
    if omni.usd.get_context().get_stage().GetPrimAtPath(stage_path):
        omni.kit.commands.execute("DeletePrims", paths=[stage_path])


def attempt_grasp(
    robot: hsr.hsr,
    controller: HSRController,
    sim_context: SimulationContext,
    z_offset: float,
) -> None:
    grasp_pose_with_offset = GRASP_POSE.copy()
    grasp_pose_with_offset["arm_lift_joint"] = GRASP_POSE["arm_lift_joint"] + z_offset
    controller.set_joint_targets(PREGRASP_POSE)
    controller.set_gripper(GRIPPER_OPEN)
    advance_simulation(sim_context, robot, 80)
    controller.set_joint_targets(grasp_pose_with_offset)
    advance_simulation(sim_context, robot, 180)
    controller.set_gripper(GRIPPER_CLOSE)
    advance_simulation(sim_context, robot, 80)
    controller.set_joint_targets(PREGRASP_POSE)
    advance_simulation(sim_context, robot, 180)


def main() -> None:
    setup_scene()
    robot = create_hsr_robot()

    kit.update()
    sim_context = SimulationContext(stage_units_in_meters=1.0)
    kit.update()
    robot.onsimulationstart(sim_context)
    sim_context.initialize_physics()
    omni.timeline.get_timeline_interface().play()

    # Warm-up so articulated views are ready.
    advance_simulation(sim_context, robot, 80)
    controller = HSRController(robot)

    model_names = list_ycb_models()
    log(f"Found {len(model_names)} YCB assets to evaluate.")

    for index, model_name in enumerate(model_names, start=1):
        log(f"[{index}/{len(model_names)}] Trying to grasp {model_name}...")
        stage_path = None
        try:
            # Compute bbox and set dynamic arm poses
            stl_path = find_model_mesh_stl(model_name)
            bbox = parse_stl_aabb(stl_path)
            log(f'Computed bbox: dx={bbox[1]-bbox[0]:.3f}, dy={bbox[3]-bbox[2]:.3f}, dz={bbox[5]-bbox[4]:.3f}')
            object_center_x = bbox[0] + (bbox[1] - bbox[0]) / 2.0
            object_center_y = bbox[2] + (bbox[3] - bbox[2]) / 2.0
            object_center_z = bbox[4] + (bbox[5] - bbox[4]) / 2.0
 
            stage_path = spawn_object(model_name, object_center_y, object_center_x)
            attempt_grasp(robot, controller, sim_context, object_center_z * 2.0)
        except Exception as error:  # noqa: BLE001 - we want to keep iterating
            log(f"Error while testing {model_name}: {error}")
        finally:
            if stage_path:
                delete_object(stage_path)

    omni.timeline.get_timeline_interface().stop()
    log("Completed YCB grasp sweep.")


if __name__ == "__main__":
    try:
        main()
    finally:
        kit.close()
