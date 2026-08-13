"""MoveGroup-action pick and place for the UR7e MoveIt configuration."""

import math
import os
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, Float64MultiArray
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener

# Must match GRIPPER_STATE_FILE in simScripts/gripper_attach.py. A plain
# file instead of a second ROS2 subscriber inside Isaac Sim's own process:
# running rclpy there repeatedly hit native crashes from mixing Isaac
# Sim's bundled rclpy build with the system one. Both this node and
# bringup.py run on the host, so a shared path works fine even though
# MoveIt/cuMotion run in Docker.
GRIPPER_STATE_FILE = '/tmp/vision_pick_gripper_state'


class PickPlaceNode(Node):
    """Use /move_action from ur_moveit_config for one detected cube at a time."""


    def __init__(self):
        super().__init__('vision_pick_place')
        for name, value in {
            'planning_group': 'ur_manipulator', 'tool_link': 'tool0', 'world_frame': 'base_link',
            # segmentation_node's z is the rod's *top surface* (that's what
            # a depth camera looking straight down actually measures, not
            # the rod's volumetric centre). MoveIt places tool0 itself, but
            # what actually needs to reach the rod is the gripper tip
            # bringup.py mounts gripper_reach (0.15m -- keep this in sync
            # with gripper_reach in bringup.py by hand, they're separate
            # processes) further out, so this is that reach length plus a
            # small margin above the surface -- close enough for
            # GripperAttacher's attach_radius (see gripper_attach.py) to
            # grab it, without the *goal* ending up in collision with the
            # table surface registered at the rod.
            'grasp_height_offset': 0.01,
            'place_x': 0.67, 'place_y': -0.60, 'place_z': 0.4,
            'retreat_height': 0.10,
            # 180deg about X: identity would leave tool0's approach axis (Z,
            # "front" per the URDF's tool0 comment) pointing along world +Z
            # (straight up). This flips it to point straight down at the
            # table for a top-down grasp instead.
            'tool_orientation_xyzw': [1.0, 0.0, 0.0, 0.0],
            # Left empty by default: ur_moveit_config's launch file already
            # decides which pipeline move_group uses (e.g. cuMotion). Set
            # this only if you need to force a specific pipeline by name.
            'planning_pipeline_id': '',
            # cuMotion/MoveIt's planning scene has no idea any rod exists
            # unless told -- segmentation_node scans every known colour
            # (not just the target) and publishes both position
            # (/detected_objects) and an estimated size
            # (/detected_object_sizes, from each blob's pixel footprint +
            # depth) for all of them. Whichever one is close to the current
            # target is excluded (it's the goal, not an obstacle) and the
            # rest are registered as collision boxes at their vision-derived
            # size -- nothing here is hand-entered.
            'rod_exclude_radius': 0.10,
            # The camera is mounted on the wrist, so it moves with the arm --
            # it's only looking at the actual workspace while the arm is
            # still near its start pose. Only scan for background obstacles
            # (table, other rods, ...) for this many seconds after startup;
            # after that, whatever was last registered stays as-is instead
            # of being replaced every 0.5s by whatever the camera currently
            # happens to be pointed at as the arm moves around to pick.
            'collision_scan_seconds': 3.0,
        }.items():
            self.declare_parameter(name, value)
        self.group = self._string('planning_group')
        self.tool_link = self._string('tool_link')
        self.world_frame = self._string('world_frame')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_action = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_pub = self.create_publisher(Bool, '/pick_place/gripper_command', 10)
        self.planning_scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.create_subscription(PoseStamped, '/detected_object_pose', self._on_detection, 10)
        self.latest_objects_msg = None
        self.create_subscription(PoseArray, '/detected_objects', self._on_objects, 10)
        self.latest_sizes_msg = None
        self.create_subscription(
            Float64MultiArray, '/detected_object_sizes', self._on_sizes, 10
        )
        self.busy = False
        self.completed = False
        self.lock = threading.Lock()
        # Highest obstacle count seen so far, used as the number of
        # collision-object slots to publish -- not tied to how many colours
        # or object types exist, so a detected surface (or anything else)
        # is just one more slot instead of something special-cased.
        self._max_tracked_objects = 0
        self.latest_target_pose = None
        # Obstacles (table, other rods, ...) were previously only ever
        # registered right before a pick actually starts -- so if nothing's
        # been picked yet (or the one-shot pick already completed), the
        # planning scene never got them at all, and RViz/MoveIt looked like
        # detection wasn't working. This keeps it up to date for a short
        # window, then freezes (see collision_scan_seconds).
        #
        # That window starts counting from the *first successful*
        # /detected_object_pose, not from node construction: at startup the
        # arm/wrist camera may still be settling into its start pose, and
        # scanning blind on a wall-clock timer risked freezing an obstacle
        # computed from wherever the camera happened to be pointed at that
        # moment (e.g. a phantom box floating above the real floor). A
        # successful detection proves the camera was actually looking at
        # the workspace.
        self._collision_scan_deadline = None
        self._collision_scan_timer = self.create_timer(0.5, self._periodic_collision_sync)
        self.get_logger().info('Ready: waiting for /detected_object_pose.')

    def _string(self, name):
        return str(self.get_parameter(name).value)

    def _number(self, name):
        return float(self.get_parameter(name).value)

    def _on_objects(self, msg):
        self.latest_objects_msg = msg

    def _on_sizes(self, msg):
        self.latest_sizes_msg = msg

    def _on_detection(self, pose):
        if self._collision_scan_deadline is None:
            # First proof the camera's actually looking at the workspace;
            # start the background-scan window from here rather than from
            # whatever the camera happened to see at node startup.
            self._collision_scan_deadline = self.get_clock().now() + Duration(
                seconds=self._number('collision_scan_seconds')
            )
        self.latest_target_pose = pose
        with self.lock:
            if self.busy or self.completed:
                return
            self.busy = True
        threading.Thread(target=self._pick_place, args=(pose,), daemon=True).start()

    def _pick_place(self, pose):
        try:
            world_pose = self._to_world(pose)
            if world_pose is None:
                return
            p = world_pose.pose.position
            self._sync_rod_collision_objects((p.x, p.y, p.z))
            place = (self._number('place_x'), self._number('place_y'), self._number('place_z'))
            grasp_target = (p.x, p.y, p.z + self._number('grasp_height_offset'))
            if not self._move('grasp', grasp_target):
                return
            self._gripper(True)
            if not self._move('place', place):
                return
            self._gripper(False)
            if self._move('retreat', (place[0], place[1], place[2] + self._number('retreat_height'))):
                self.completed = True
        finally:
            with self.lock:
                self.busy = False

    def _to_world(self, pose):
        if pose.header.frame_id == self.world_frame:
            return pose

        candidate_frames = [self.world_frame]
        for frame in ('base_link', 'world', 'map', 'base', 'odom', 'tool0'):
            if frame not in candidate_frames:
                candidate_frames.append(frame)

        try:
            # all_frames_as_string() returns prose lines like "Frame X exists
            # with parent Y." meant for humans; the actual frame ids are the
            # top-level keys of all_frames_as_yaml().
            frames_yaml = self.tf_buffer.all_frames_as_yaml()
            available_frames = set((yaml.safe_load(frames_yaml) or {}).keys())
        except Exception as exc:
            available_frames = set()
            self.get_logger().warn(f'TF frame query failed: {exc}')

        if not available_frames:
            self.get_logger().warn('TF tree is empty or not available yet; waiting for frames')
        else:
            self.get_logger().info(f'TF frames available: {sorted(available_frames)}')

        # Try the well-known candidate names directly rather than gating on
        # `available_frames`: all_frames_as_yaml() only lists frames that
        # have a parent, so a tree root (e.g. "world") never appears as a
        # key even though lookup_transform resolves it just fine. Filtering
        # candidates by that set would silently skip valid root frames.
        ordered_frames = list(candidate_frames)
        for frame in sorted(available_frames):
            if frame not in ordered_frames:
                ordered_frames.append(frame)

        last_error = None

        for frame in ordered_frames:
            if frame == pose.header.frame_id:
                continue
            try:
                transform = self.tf_buffer.lookup_transform(
                    frame, pose.header.frame_id, rclpy.time.Time(), Duration(seconds=2.0)
                )
                self.world_frame = frame
                return do_transform_pose_stamped(pose, transform)
            except TransformException as exc:
                last_error = exc

        self.get_logger().error(f'TF conversion to world failed: {last_error}')
        return None

    def _move(self, name, xyz):
        if not self.move_action.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('No /move_action. Start ur_moveit_config first.')
            return False

        for attempt in range(3):
            relaxed = attempt > 0
            target = xyz
            if relaxed:
                target = (target[0], target[1], target[2] + 0.03)
            self.get_logger().info(f'{name}: {tuple(round(v, 3) for v in target)} (attempt {attempt + 1})')
            ok = self._move_once(name, target, relaxed=relaxed)
            if ok:
                return True
            if attempt < 2:
                self.get_logger().warning(f'{name} attempt {attempt + 1} failed; retrying with looser constraints')
        return False

    def _move_once(self, name, xyz, relaxed=False):
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group
        pipeline_id = self._string('planning_pipeline_id')
        if pipeline_id:
            goal.request.pipeline_id = pipeline_id
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 20.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2
        goal.request.goal_constraints = [self._constraints(xyz, relaxed=relaxed)]
        goal.planning_options.plan_only = False
        send = self.move_action.send_goal_async(goal)
        send_event = threading.Event(); send.add_done_callback(lambda _: send_event.set()); send_event.wait()
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'{name} goal rejected')
            return False
        result = handle.get_result_async()
        result_event = threading.Event(); result.add_done_callback(lambda _: result_event.set()); result_event.wait()
        response = result.result().result
        ok = response.error_code.val == response.error_code.SUCCESS
        if not ok:
            self.get_logger().error(f'{name} failed with MoveIt code {response.error_code.val}')
        return ok

    def _normalize_tool_orientation(self, orientation):
        if orientation is None:
            return (0.0, 0.0, 0.0, 1.0)
        try:
            q = tuple(float(value) for value in orientation)
        except TypeError:
            return (0.0, 0.0, 0.0, 1.0)
        if len(q) != 4:
            return (0.0, 0.0, 0.0, 1.0)
        norm = math.sqrt(sum(value * value for value in q))
        if norm < 1e-9:
            return (0.0, 0.0, 0.0, 1.0)
        q = tuple(value / norm for value in q)
        if q[3] < 0.0:
            q = tuple(-value for value in q)
        return q

    def _constraints(self, xyz, relaxed=False):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, xyz)
        q = self._normalize_tool_orientation(self.get_parameter('tool_orientation_xyzw').value)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
        position = PositionConstraint(header=PoseStamped().header, link_name=self.tool_link, weight=1.0)
        position.header.frame_id = self.world_frame
        box_size = 0.08 if relaxed else 0.05
        position.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[box_size] * 3)]
        position.constraint_region.primitive_poses = [pose]
        orientation_constraint = OrientationConstraint(
            link_name=self.tool_link, orientation=pose.orientation, weight=0.0 if relaxed else 1.0
        )
        orientation_constraint.header.frame_id = self.world_frame
        tolerance = 0.7 if relaxed else 0.35
        orientation_constraint.absolute_x_axis_tolerance = tolerance
        orientation_constraint.absolute_y_axis_tolerance = tolerance
        orientation_constraint.absolute_z_axis_tolerance = tolerance
        return Constraints(position_constraints=[position], orientation_constraints=[orientation_constraint])

    def _gripper(self, close):
        self.gripper_pub.publish(Bool(data=close))
        self.get_logger().info('gripper ' + ('close' if close else 'open'))
        try:
            tmp_path = GRIPPER_STATE_FILE + '.tmp'
            with open(tmp_path, 'w') as state_file:
                state_file.write('1' if close else '0')
            os.replace(tmp_path, GRIPPER_STATE_FILE)  # atomic, so bringup.py never reads a half-write
        except OSError as exc:
            self.get_logger().warning(f'Could not write gripper state file: {exc}')

    def _other_detected_objects(self, target, exclude_radius):
        """World-frame (position, size) of every detected object except the
        one near `target` (that one's the pick goal, not an obstacle).
        Size comes from /detected_object_sizes -- each blob's own pixel
        footprint + measured depth -- not a hand-entered guess."""
        objects_msg = self.latest_objects_msg
        sizes_msg = self.latest_sizes_msg
        if objects_msg is None or sizes_msg is None:
            return []

        sizes = sizes_msg.data
        if len(sizes) != len(objects_msg.poses) * 3:
            # Position and size are published as two separate messages from
            # the same camera frame; skip rather than risk pairing a pose
            # with another object's size if one topic is momentarily behind.
            self.get_logger().warn('Detected objects/sizes counts out of sync; skipping obstacles this cycle.')
            return []

        results = []
        for i, pose in enumerate(objects_msg.poses):
            stamped = PoseStamped()
            stamped.header = objects_msg.header
            stamped.pose = pose
            world = self._to_world(stamped)
            if world is None:
                continue
            p = world.pose.position
            if math.dist((p.x, p.y, p.z), target) <= exclude_radius:
                continue
            results.append(((p.x, p.y, p.z), tuple(sizes[i * 3:i * 3 + 3])))
        return results

    def _build_collision_scene(self, target):
        """Register every detected object except the current target as a
        collision obstacle -- rods, a detected support surface, whatever
        else shows up. Slot ids run up to the most objects ever seen at
        once, so a previous cycle's obstacle that isn't seen this time gets
        explicitly removed instead of lingering as a phantom."""
        exclude_radius = self._number('rod_exclude_radius')
        obstacles = self._other_detected_objects(target, exclude_radius)
        self._max_tracked_objects = max(self._max_tracked_objects, len(obstacles))

        objects = []
        for i in range(self._max_tracked_objects):
            obj = CollisionObject()
            obj.id = f'detected_object_{i}'
            obj.header.frame_id = self.world_frame
            if i < len(obstacles):
                position, size = obstacles[i]
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = position
                pose.orientation.w = 1.0
                obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size))]
                obj.primitive_poses = [pose]
                obj.operation = CollisionObject.ADD
            else:
                obj.operation = CollisionObject.REMOVE
            objects.append(obj)

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = objects
        return scene

    def _sync_rod_collision_objects(self, target):
        """Called right before a pick sequence starts moving. Publishes a
        few times over ~0.6s to be confident move_group applied it before
        planning starts; runs on the per-pick background thread, so
        blocking here is fine (unlike _periodic_collision_sync, which runs
        on the node's single-threaded executor)."""
        scene = self._build_collision_scene(target)
        for _ in range(3):
            self.planning_scene_pub.publish(scene)
            time.sleep(0.2)

    def _periodic_collision_sync(self):
        """Keep the planning scene's obstacles current for a short window
        after startup, not just while a pick is actively running --
        otherwise the table/rods never show up in RViz/MoveIt until (and
        unless) a pick happens to be in progress. Stops after
        collision_scan_seconds: the camera is mounted on the wrist, so once
        the arm starts moving around to pick, it's no longer looking at the
        workspace in general -- it should stop replacing the registered
        obstacles with whatever it happens to be pointed at moment to
        moment. Uses whatever target was last detected purely to decide
        what to exclude; if none has ever been seen, nothing is excluded
        and every detected object is treated as an obstacle."""
        if self._collision_scan_deadline is None:
            return  # no proven-good camera view yet; nothing to scan from

        if self.busy:
            # A pick is actively planning/executing right now -- publishing a
            # new scene diff mid-motion is what MoveIt reports back as
            # MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE. Skip this cycle;
            # _sync_rod_collision_objects already synced the scene once at
            # the start of the pick.
            return

        if self.get_clock().now() >= self._collision_scan_deadline:
            self.get_logger().info(
                'Background obstacle scan window elapsed; freezing detected obstacles.'
            )
            self._collision_scan_timer.cancel()
            return

        target = (1e9, 1e9, 1e9)
        if self.latest_target_pose is not None:
            world_pose = self._to_world(self.latest_target_pose)
            if world_pose is not None:
                p = world_pose.pose.position
                target = (p.x, p.y, p.z)
        self.planning_scene_pub.publish(self._build_collision_scene(target))


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
