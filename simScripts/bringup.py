"""
bringup.py

One-command Isaac Sim bringup

Isaac Sim 6.0.1
"""

from isaacsim import SimulationApp

# ---------------------------------------------------------------------
# Isaac Sim MUST be initialized before importing other Isaac modules
# ---------------------------------------------------------------------

simulation_app = SimulationApp(
    {
        "headless": False,
    }
)
# ======== 在这里加载 Extension ========

import omni.kit.app

ext_manager = omni.kit.app.get_app().get_extension_manager()

ext_manager.set_extension_enabled_immediate(
    "isaacsim.ros2.bridge",
    True,
)

ext_manager.set_extension_enabled_immediate(
    "isaacsim.ros2.nodes",
    True,
)

print(
    "ROS2 Bridge:",
    ext_manager.is_extension_enabled("isaacsim.ros2.bridge"),
)

print(
    "ROS2 Nodes:",
    ext_manager.is_extension_enabled("isaacsim.ros2.nodes"),
)

# ---------------------------------------------------------------------
# Imports after SimulationApp
# ---------------------------------------------------------------------

from isaacsim.core.api import World

from scene_builder import SceneBuilder
from robot_loader import RobotLoader
from camera_utils import CameraUtils
from ros2_cam import ROS2CameraPublisher


def main():

    print("=" * 60)
    print("Creating Isaac Sim world...")
    print("=" * 60)

    world = World(stage_units_in_meters=1.0)

    #
    # Build environment
    #
    builder = SceneBuilder(world)
    builder.build()

    #
    # Load robot
    #
    loader = RobotLoader()

    robot_path = loader.load(
        urdf_path="./assets/robots/ur7e/ur7e.urdf",
        robot_name="UR7e",
        position=(0.0, 0.0, 0.0),
        fix_base=True,
    )

    print(f"Robot loaded at {robot_path}")

    #
    # Camera
    #
    camera = CameraUtils()

    camera_path = camera.create_camera(
        name="MainCamera",
        position=(1.2, -1.2, 1.2),
        rotation=(55.0, 0.0, 45.0),
    )

    #
    # ROS2 Camera Publisher
    #
    ros = ROS2CameraPublisher()

    ros.publish_rgb(
        "/ROS2/RGB",
        "/World/MainCamera",
    )

    ros.publish_depth(
        "/ROS2/Depth",
        "/World/MainCamera",
    )

    ros.publish_camera_info(
        "/ROS2/CameraInfo",
    )
    #
    # Initialize simulation
    #
    world.reset()

    print("=" * 60)
    print("Simulation started.")
    print("=" * 60)

    #
    # Main Loop
    #
    while simulation_app.is_running():

        world.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    main()