# Copyright (c) 2023, Toyota Motor Corporation
# Copyright (c) 2023, MID Academic Promotions, Inc.
# All rights reserved.

from isaacsim.simulation_app import SimulationApp
import os
from hsr_omniverse2_sim.paths import USD_DIR
import numpy as np

kit = SimulationApp({"renderer": "RayTracedLighting", "headless": False})
#kit = SimulationApp({"renderer": "PathTracing", "headless": False})

import omni.ui
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils import viewports, stage
from isaacsim.storage.native import get_assets_root_path
import omni.kit.commands
from omni.isaac.dynamic_control import _dynamic_control
from isaacsim.core.utils.prims import create_prim

viewports.set_camera_view(eye=np.array([1.2, 1.2, 0.8]), target=np.array([0, 0, 0.5]))

# Loading the simple_room environment
assets_root_path = get_assets_root_path()
BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Grid/default_environment.usd"
stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)

grid = range(3)

for x in grid:
    for y in grid:
        path = f"/world_{x}_{y}/hsr"
        create_prim(prim_path=path, prim_type="Xform", position=[x, y, 0.0])
        hsr = stage.add_reference_to_stage(os.path.join(USD_DIR, "hsrb", "hsrb4s.usd"), path)

omni.timeline.get_timeline_interface().play()
kit.update()
simulation_context = SimulationContext(stage_units_in_meters=1.0)
kit.update()

dc = _dynamic_control.acquire_dynamic_control_interface()

joints = []
for x in grid:
    for y in grid:
        path = f"/world_{x}_{y}/hsr"
        art = dc.get_articulation(path)
        joints.append(dc.find_articulation_dof(art, "arm_flex_joint"))

simulation_context.initialize_physics()
simulation_context.play()

kit.update()

while kit.is_running():
    # Run with a fixed step size
    simulation_context.step(render=True)
    tgt = simulation_context.current_time % 2.0 / 2.0
    for j in joints:
        dc.set_dof_position_target(j, tgt)
    print(simulation_context.current_time)

simulation_context.stop()
kit.close()
