"""Shared RGB-D perception plumbing.

segmentation_node.py (HSV colour blobs) and yolo_detection_node.py
(pretrained YOLO segmentation) both need the exact same downstream
handling once they've found something in a 2D image: CameraInfo
intrinsics, depth back-projection of a detected pixel into a 3D pose,
generic table/box surface detection, and the /detected_object_pose +
/detected_objects + /detected_object_sizes publishing contract
pick_place_node.py consumes. This module holds that common part once;
each node only implements *how* it finds 2D detections in a frame
(colour thresholding vs a neural network), not what happens to a
detection afterward -- which is exactly the "generic pixel -> robot-frame
mapping, decoupled from whatever does the recognising" split the project
is meant to have.
"""

from dataclasses import dataclass
from typing import List, Optional

import message_filters
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray

from .depth_processor import depth_at_pixel, project_pixel_to_3d
from .surface_detector import detect_surfaces


@dataclass
class Detection2D:
    """One detected object instance in a single RGB-D frame, in pixel
    coordinates -- however it was found."""

    u: int
    v: int
    bbox_w_px: float
    bbox_h_px: float
    is_target: bool = False
    # Full-image binary mask (0/255), only needed for whichever instance
    # is_target -- used purely for the /vision_pick/mask visualisation, so
    # subclasses can skip building it for non-target instances.
    mask: Optional[np.ndarray] = None


class RGBDPerceptionNode(Node):
    """Base class for an RGB-D object-localization node. Subclasses
    implement `_detect_instances`; this handles intrinsics, depth
    back-projection, generic surface detection, and publishing."""

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.declare_parameter('depth_window_radius', 2)
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        # Whatever flat surfaces the camera is looking down at (a table, a
        # box sitting on it, a shelf, ...) get detected from depth alone,
        # the same way for any of them, and fed into
        # /detected_objects/_sizes as just more entries -- no surface is
        # ever hand-positioned or special-cased. min_area_fraction=0.03
        # (not the 0.15 a single-dominant-table scene could get away with):
        # once more than one surface/object is in frame -- and once the
        # wider horizontal_fov_deg needed for full-table coverage shrinks
        # every surface's relative pixel footprint further -- a high area
        # threshold means only the floor (large, unobstructed) ever clears
        # it while the table/box never register at all.
        self.declare_parameter('surface_depth_tolerance', 0.03)
        self.declare_parameter('surface_min_area_fraction', 0.03)
        self.declare_parameter('surface_thickness', 0.05)
        self.declare_parameter('surface_max_count', 10)

        self.camera_info: Optional[CameraInfo] = None
        self.bridge = CvBridge()

        self.pixel_pub = self.create_publisher(Point, '/detected_pixel', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/detected_object_pose', 10)
        self.mask_pub = self.create_publisher(Image, '/vision_pick/mask', 10)
        # Every recognised instance, target included, so pick_place_node
        # can treat whichever ones aren't the current target as obstacles.
        self.objects_pub = self.create_publisher(PoseArray, '/detected_objects', 10)
        self.object_sizes_pub = self.create_publisher(Float64MultiArray, '/detected_object_sizes', 10)

        info_topic = str(self.get_parameter('camera_info_topic').value)
        self.create_subscription(CameraInfo, info_topic, self._camera_info_callback, 10)

        rgb_topic = str(self.get_parameter('rgb_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        self.rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.10
        )
        self.sync.registerCallback(self._rgbd_callback)

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def _intrinsics(self):
        assert self.camera_info is not None
        k = self.camera_info.k
        return k[0], k[4], k[2], k[5]

    def _detect_instances(self, bgr: np.ndarray, depth: np.ndarray) -> List[Detection2D]:
        """Return every detected object instance in this frame, with at
        most one marked is_target=True. Implemented by subclasses."""
        raise NotImplementedError

    def _rgbd_callback(self, rgb_msg: Image, depth_msg: Image) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warning(f'Cannot process RGB frame: {exc}')
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except CvBridgeError as exc:
            self.get_logger().warning(f'Cannot process depth frame: {exc}')
            return

        if self.camera_info is None:
            self.get_logger().debug('Waiting for CameraInfo before publishing poses.')
            return
        if depth.shape[:2] != bgr.shape[:2]:
            self.get_logger().warning('RGB and depth resolutions differ; cannot use same pixel coordinates.')
            return

        scale = 0.001 if depth_msg.encoding.upper() == '16UC1' else 1.0
        window_radius = int(self.get_parameter('depth_window_radius').value)
        depth_arr = np.asarray(depth)

        header = rgb_msg.header
        header.frame_id = self.camera_info.header.frame_id or rgb_msg.header.frame_id

        objects = PoseArray()
        objects.header = header
        sizes = []
        fx, fy, cx, cy = self._intrinsics()

        target = None  # (pose, mask, u, v)
        for det in self._detect_instances(bgr, depth_arr):
            z = depth_at_pixel(depth_arr, det.u, det.v, scale, window_radius)
            if z is None:
                continue
            try:
                x, y, z = project_pixel_to_3d(det.u, det.v, z, fx, fy, cx, cy)
            except ValueError as exc:
                self.get_logger().warning(str(exc))
                continue

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.w = 1.0
            objects.poses.append(pose)

            # Pixel footprint -> metric size using similar triangles (size /
            # z = size_px / focal_px). The camera looks down at these
            # objects, so their vertical (table-normal) extent never shows
            # up in either image dimension; approximated with the narrower
            # measured side.
            size_x = (det.bbox_w_px / fx) * z
            size_y = (det.bbox_h_px / fy) * z
            sizes.extend((size_x, size_y, min(size_x, size_y)))

            if det.is_target:
                target = (pose, det.mask, det.u, det.v)

        # Whatever the objects are sitting on -- table, box, shelf, ... --
        # detected generically from depth alone, independent of whichever
        # detection front-end is running.
        surfaces = detect_surfaces(
            depth_arr,
            depth_scale=scale,
            tolerance=float(self.get_parameter('surface_depth_tolerance').value),
            min_area_fraction=float(self.get_parameter('surface_min_area_fraction').value),
            max_surfaces=int(self.get_parameter('surface_max_count').value),
        )
        for u, v, surface_z, (bbox_w_px, bbox_h_px) in surfaces:
            try:
                x, y, z = project_pixel_to_3d(u, v, surface_z, fx, fy, cx, cy)
            except ValueError as exc:
                self.get_logger().warning(str(exc))
                continue
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.w = 1.0
            objects.poses.append(pose)
            sizes.extend((
                (bbox_w_px / fx) * z,
                (bbox_h_px / fy) * z,
                float(self.get_parameter('surface_thickness').value),
            ))

        self.objects_pub.publish(objects)
        self.object_sizes_pub.publish(Float64MultiArray(data=sizes))

        if target is not None:
            pose, mask, u, v = target
            pose_stamped = PoseStamped()
            pose_stamped.header = header
            pose_stamped.pose = pose
            self.pose_pub.publish(pose_stamped)
            if mask is not None:
                mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
                mask_msg.header = rgb_msg.header
                self.mask_pub.publish(mask_msg)
            self.pixel_pub.publish(Point(x=float(u), y=float(v), z=0.0))
