from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

print("Creating world...")

world = World()

print("Adding ground...")
world.scene.add_default_ground_plane()

print("Adding cube...")
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/TestCube",
        name="cube",
        position=np.array([0.0, 0.0, 1.0]),
        scale=np.array([0.5, 0.5, 0.5]),
        color=np.array([1.0, 0.0, 0.0]),
    )
)

print("Reset world...")
world.reset()

print("Simulation running...")

while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()