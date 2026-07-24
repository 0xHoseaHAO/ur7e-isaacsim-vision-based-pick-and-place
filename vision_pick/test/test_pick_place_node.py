import types

from geometry_msgs.msg import Pose, PoseArray
from moveit_msgs.msg import CollisionObject
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray

from vision_pick.pick_place_node import PickPlaceNode


def _fake_node(params, max_tracked_objects=0, collision_scan_expired=False):
    node = PickPlaceNode.__new__(PickPlaceNode)
    node.get_parameter = lambda name: types.SimpleNamespace(value=params[name])
    node.get_logger = lambda: types.SimpleNamespace(warn=lambda *_: None, info=lambda *_: None)
    node._max_tracked_objects = max_tracked_objects
    node.latest_objects_msg = None
    node.latest_sizes_msg = None
    node.latest_target_pose = None
    # get_clock().now() is pinned to t=0; the deadline sits either just
    # before it (expired) or far after (still within the scan window).
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: Time(seconds=0))
    node._collision_scan_deadline = Time(seconds=0 if collision_scan_expired else 1e9)
    node._collision_scan_timer = types.SimpleNamespace(cancel=lambda: None)
    return node


def test_normalize_orientation_normalizes_and_flips_sign():
    node = PickPlaceNode.__new__(PickPlaceNode)

    normalized = node._normalize_tool_orientation([0.0, 2.0, 0.0, 0.0])

    assert normalized == (0.0, 1.0, 0.0, 0.0)


def test_normalize_orientation_uses_identity_for_zero_input():
    node = PickPlaceNode.__new__(PickPlaceNode)

    normalized = node._normalize_tool_orientation([0.0, 0.0, 0.0, 0.0])

    assert normalized == (0.0, 0.0, 0.0, 1.0)


def test_sync_rod_collision_objects_excludes_the_target_rod(monkeypatch):
    monkeypatch.setattr('vision_pick.pick_place_node.time.sleep', lambda _: None)

    # Seeded at 2 to simulate "a previous cycle saw 2 obstacles" -- this
    # cycle only finds 1, so slot 1 should come back as an explicit REMOVE
    # instead of just not being mentioned.
    node = _fake_node({'rod_exclude_radius': 0.12}, max_tracked_objects=2)
    node.world_frame = 'base_link'

    # Vision reports two blobs: one at the target and one elsewhere. Both
    # are already in the world frame so _to_world's early-return path
    # (frame_id already matches) applies without needing a real TF buffer.
    objects = PoseArray()
    objects.header.frame_id = 'base_link'
    target_pose = Pose()
    target_pose.position.x, target_pose.position.y, target_pose.position.z = 0.5, 0.0, 0.42
    other_pose = Pose()
    other_pose.position.x, other_pose.position.y, other_pose.position.z = 0.75, -0.1, 0.42
    objects.poses = [target_pose, other_pose]
    node.latest_objects_msg = objects
    node.latest_sizes_msg = Float64MultiArray(data=[0.20, 0.05, 0.05, 0.25, 0.06, 0.06])

    published = []
    node.planning_scene_pub = types.SimpleNamespace(publish=published.append)

    node._sync_rod_collision_objects((0.5, 0.0, 0.42))

    scene = published[-1]
    ops = {obj.id: obj.operation for obj in scene.world.collision_objects}
    assert ops['detected_object_0'] == CollisionObject.ADD  # the other rod, kept as an obstacle
    assert ops['detected_object_1'] == CollisionObject.REMOVE  # nothing else detected this cycle

    kept = next(obj for obj in scene.world.collision_objects if obj.id == 'detected_object_0')
    assert list(kept.primitives[0].dimensions) == [0.25, 0.06, 0.06]  # the other rod's own estimated size


def test_sync_rod_collision_objects_with_no_detections_yet(monkeypatch):
    monkeypatch.setattr('vision_pick.pick_place_node.time.sleep', lambda _: None)

    node = _fake_node({'rod_exclude_radius': 0.12}, max_tracked_objects=3)
    node.world_frame = 'base_link'
    node.latest_objects_msg = None
    node.latest_sizes_msg = None

    published = []
    node.planning_scene_pub = types.SimpleNamespace(publish=published.append)

    node._sync_rod_collision_objects((0.5, 0.0, 0.42))

    ops = {obj.id: obj.operation for obj in published[-1].world.collision_objects}
    assert len(ops) == 3
    assert all(op == CollisionObject.REMOVE for op in ops.values())


def test_sync_rod_collision_objects_skips_when_counts_out_of_sync(monkeypatch):
    monkeypatch.setattr('vision_pick.pick_place_node.time.sleep', lambda _: None)

    node = _fake_node({'rod_exclude_radius': 0.12}, max_tracked_objects=2)
    node.world_frame = 'base_link'

    objects = PoseArray()
    objects.header.frame_id = 'base_link'
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = 0.75, -0.1, 0.42
    objects.poses = [pose]
    node.latest_objects_msg = objects
    node.latest_sizes_msg = Float64MultiArray(data=[0.20, 0.05, 0.05, 0.25, 0.06, 0.06])  # sizes for 2, poses for 1

    published = []
    node.planning_scene_pub = types.SimpleNamespace(publish=published.append)

    node._sync_rod_collision_objects((0.5, 0.0, 0.42))

    ops = {obj.id: obj.operation for obj in published[-1].world.collision_objects}
    assert len(ops) == 2
    assert all(op == CollisionObject.REMOVE for op in ops.values())


def test_periodic_collision_sync_runs_without_an_active_pick():
    # Regression test: obstacles used to only ever get registered from
    # inside _pick_place, so they never appeared in the planning scene
    # unless a pick was actively in progress. _periodic_collision_sync must
    # work on its own, driven only by the last detected target (if any).
    node = _fake_node({'rod_exclude_radius': 0.12})
    node.world_frame = 'base_link'

    objects = PoseArray()
    objects.header.frame_id = 'base_link'
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = 0.75, -0.1, 0.42
    objects.poses = [pose]
    node.latest_objects_msg = objects
    node.latest_sizes_msg = Float64MultiArray(data=[0.20, 0.05, 0.05])
    node.latest_target_pose = None  # nothing picked (or being picked) yet

    published = []
    node.planning_scene_pub = types.SimpleNamespace(publish=published.append)

    node._periodic_collision_sync()

    assert len(published) == 1  # a single publish, no retry-sleep loop
    ops = {obj.id: obj.operation for obj in published[-1].world.collision_objects}
    assert ops['detected_object_0'] == CollisionObject.ADD  # nothing to exclude, so it's an obstacle


def test_periodic_collision_sync_does_nothing_before_any_real_detection():
    # The scan window used to start counting from node construction, so a
    # camera that hadn't yet settled into a real view of the workspace
    # could get scanned (and later frozen) anyway -- e.g. a phantom box
    # floating above the actual floor. It must not scan at all until
    # _on_detection has proven the camera saw something real.
    node = _fake_node({'rod_exclude_radius': 0.12})
    node._collision_scan_deadline = None  # no successful detection yet

    objects = PoseArray()
    objects.header.frame_id = 'base_link'
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = 0.75, -0.1, 0.42
    objects.poses = [pose]
    node.latest_objects_msg = objects
    node.latest_sizes_msg = Float64MultiArray(data=[0.20, 0.05, 0.05])

    published = []
    node.planning_scene_pub = types.SimpleNamespace(publish=published.append)

    node._periodic_collision_sync()

    assert published == []


def test_periodic_collision_sync_freezes_after_the_scan_window_elapses():
    # The eye-in-hand camera moves with the arm, so it must stop replacing
    # registered obstacles with whatever it's currently pointed at once the
    # arm starts moving around -- otherwise RViz fills up with collision
    # boxes chasing the wrist around instead of a stable scene.
    node = _fake_node({'rod_exclude_radius': 0.12}, collision_scan_expired=True)
    node.world_frame = 'base_link'

    objects = PoseArray()
    objects.header.frame_id = 'base_link'
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = 0.75, -0.1, 0.42
    objects.poses = [pose]
    node.latest_objects_msg = objects
    node.latest_sizes_msg = Float64MultiArray(data=[0.20, 0.05, 0.05])

    published = []
    node.planning_scene_pub = types.SimpleNamespace(publish=published.append)
    cancelled = []
    node._collision_scan_timer = types.SimpleNamespace(cancel=lambda: cancelled.append(True))

    node._periodic_collision_sync()

    assert published == []  # frozen: nothing (re-)published once the window's elapsed
    assert cancelled == [True]  # and the timer stops firing entirely
