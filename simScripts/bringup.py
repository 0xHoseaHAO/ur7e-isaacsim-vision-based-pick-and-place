"""
bringup.py

One-command Isaac Sim bringup
Isaac Sim 6.0.1
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": False,
    }
)

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

print("ROS2 Bridge:",
      ext_manager.is_extension_enabled("isaacsim.ros2.bridge"))

print("ROS2 Nodes:",
      ext_manager.is_extension_enabled("isaacsim.ros2.nodes"))

# --------------------------------------------------------

from isaacsim.core.api import World

from scene_builder import SceneBuilder
from robot_loader import RobotLoader
from camera_utils import CameraUtils
from ros2_cam import ROS2CameraPublisher
from ros2_joint_command import create_joint_command_graph
from ros2_moveit import create_moveit_action_graph
from publish_joint_states import create_joint_state_publisher


def main():

    print("=" * 60)
    print("Creating Isaac Sim world...")
    print("=" * 60)

    world = World(stage_units_in_meters=1.0)

    #
    # Scene
    #
    SceneBuilder(world).build()

    #
    # Robot
    #
    loader = RobotLoader()

    robot = loader.load(
        urdf_path="./assets/robots/ur7e/ur7e.urdf",
        robot_name="UR7e",
        position=(0, 0, 0),
        fix_base=True,
    )

    print("Robot loaded.")

    #
    # Camera
    #
    camera = CameraUtils()

    camera.create_camera(
        name="MainCamera",
        position=(1.2, -1.2, 1.2),
        rotation=(55.0, 0.0, 45.0),
    )

    ros = ROS2CameraPublisher()

    ros.publish_rgb(
        "/ROS2/RGB",
        "/World/MainCamera",
    )

    ros.publish_depth(
        "/ROS2/Depth",
        "/World/MainCamera",
    )

    #
    # Joint State Publisher
    #
    create_joint_state_publisher(
        graph_path="/ROS2/JointState",
        robot_prim="/World/UR7e/Geometry/world/base_link",
        topic="/joint_states",
    )

    #
    # Initialize
    #
    world.reset()
    robot.initialize()

    print("=" * 60)
    print("Robot initialized")
    print("=" * 60)

    print("DOF:", robot.num_dof)
    print("Joint Names:", robot.dof_names)

    #
    # MoveIt
    #
    create_moveit_action_graph(
        robot_prim="/World/UR7e/Geometry/world/base_link",
    )

    #
    # Joint Command
    #
    create_joint_command_graph(
        robot_prim="/World/UR7e/Geometry/world/base_link",
        topic="/joint_command",
    )

    print("=" * 60)
    print("Simulation started")
    print("=" * 60)

    frame = 0

    while simulation_app.is_running():

        world.step(render=True)

        frame += 1

        if frame % 120 == 0:
            print("Frame:", frame)
            print(robot.get_joint_positions())

    simulation_app.close()


if __name__ == "__main__":
    main()