#!/usr/bin/env python3
"""gripper_bridge.py

Standalone bridge: subscribes to /pick_place/gripper_command and writes
'0'/'1' to GRIPPER_STATE_FILE, the same file gripper_attach.py polls from
inside Isaac Sim's process.

Why this exists as a separate script instead of pick_place_node.py's own
direct file write being enough: pick_place_node.py's Docker container
(the isaac_ros_dev container that runs move_group/cuMotion) is started
with `--network host` but NO /tmp bind mount, so the container's /tmp and
Isaac Sim's (host) /tmp are two different filesystems -- if
pick_place_node runs inside that container, its writes to
/tmp/vision_pick_gripper_state are invisible to gripper_attach.py on the
host, and the gripper silently never grabs anything (no error either,
since the file just doesn't exist from gripper_attach's side -- it reads
as "open" forever).

This script runs directly on the host (same machine as Isaac Sim, sharing
its real /tmp) in its own plain Python process with a normal system ROS2
install -- never imported into Isaac Sim's own interpreter, so it can't
hit the bundled-vs-system rclpy conflict that crashed things when rclpy
ran inside Isaac Sim's process directly. It just needs ROS2 humble
sourced; run it in its own terminal (not the one running bringup.py):

    source /opt/ros/humble/setup.bash
    source install/setup.bash   # from the vision_pick workspace, for the msg types
    python3 simScripts/gripper_bridge.py
"""

import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# Must match GRIPPER_STATE_FILE in gripper_attach.py and pick_place_node.py.
GRIPPER_STATE_FILE = '/tmp/vision_pick_gripper_state'


class GripperBridge(Node):

    def __init__(self):
        super().__init__('gripper_bridge')
        self.create_subscription(Bool, '/pick_place/gripper_command', self._on_command, 10)
        self.get_logger().info(f'Bridging /pick_place/gripper_command -> {GRIPPER_STATE_FILE}')

    def _on_command(self, msg):
        tmp_path = GRIPPER_STATE_FILE + '.tmp'
        with open(tmp_path, 'w') as state_file:
            state_file.write('1' if msg.data else '0')
        os.replace(tmp_path, GRIPPER_STATE_FILE)  # atomic, so readers never see a half-write
        self.get_logger().info('gripper state file <- ' + ('close' if msg.data else 'open'))


def main(args=None):
    rclpy.init(args=args)
    node = GripperBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
