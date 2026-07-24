"""ROS 2 node: pretrained YOLO instance segmentation for object localization.

Swaps segmentation_node.py's HSV colour matching for a stock, pretrained
Ultralytics YOLO segmentation checkpoint -- no training or fine-tuning here,
just whatever the off-the-shelf model (COCO's 80 classes for any stock
checkpoint) already recognises. Shares the exact same intrinsics/depth
back-projection/surface-detection/publishing plumbing as
segmentation_node.py via perception_base.RGBDPerceptionNode, so
pick_place_node.py needs no changes at all to consume this node's output
instead.
"""

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image

from .perception_base import Detection2D, RGBDPerceptionNode

try:
    from ultralytics import YOLO
except ImportError as exc:  # pragma: no cover - environment guard
    YOLO = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


class YoloDetectionNode(RGBDPerceptionNode):
    """Detect objects with a pretrained YOLO segmentation model and recover
    each instance's 3D position, the same way segmentation_node.py did for
    HSV colour blobs."""

    def __init__(self) -> None:
        super().__init__('vision_yolo_detection')
        if YOLO is None:
            raise RuntimeError(
                f'ultralytics is not installed ({_IMPORT_ERROR}); run `pip install ultralytics`.'
            )

        # 'm' (medium) here trades size for accuracy versus the 'n' (nano)
        # checkpoint -- fine for developing/testing on a workstation GPU,
        # but reconsider before an eventual Jetson deployment (the earlier
        # nano choice was specifically sized for that; a Jetson Orin
        # Nano-class board may not run 'm' in real time). Ultralytics
        # downloads the checkpoint automatically on first use if not
        # already cached locally.
        self.declare_parameter('model_path', 'yolo26m-seg.pt')
        # Empty string = no specific target: whichever detection has the
        # highest confidence this frame becomes the pick target. Otherwise
        # matched as a case-insensitive substring against the model's class
        # names -- COCO's 80 classes for the stock checkpoint, so objects
        # outside that vocabulary (e.g. this project's synthetic colour
        # rods) won't be recognised without separately fine-tuning.
        self.declare_parameter('target_class', '')
        self.declare_parameter('confidence_threshold', 0.4)

        model_path = str(self.get_parameter('model_path').value)
        self.get_logger().info(f'Loading YOLO model: {model_path} (downloads on first use if not cached)')
        self.model = YOLO(model_path)
        self.class_names = self.model.names  # {id: name}

        self.target_class = str(self.get_parameter('target_class').value).lower()
        self.add_on_set_parameters_callback(self._on_parameter_change)
        target_desc = self.target_class or '(highest-confidence detection each frame)'
        self.get_logger().info(f'YOLO target class: {target_desc}; model: {model_path}')

        # Every detection this frame (boxes + masks + class + confidence),
        # drawn by Ultralytics' own annotator -- add this as an Image
        # display in RViz2, or `ros2 run rqt_image_view rqt_image_view`, to
        # actually see what the model is finding instead of only the
        # target's silhouette on /vision_pick/mask.
        self.debug_image_pub = self.create_publisher(Image, '/vision_pick/yolo_debug', 10)

    def _on_parameter_change(self, parameters):
        for parameter in parameters:
            if parameter.name == 'target_class':
                self.target_class = str(parameter.value).lower()
                self.get_logger().info(
                    f'Target class changed to {self.target_class or "(highest confidence)"}'
                )
        return SetParametersResult(successful=True)

    @staticmethod
    def _instance_footprint(polygon: np.ndarray):
        """(centroid_u, centroid_v, bbox_w_px, bbox_h_px, contour) from a
        YOLO segmentation polygon -- same moments/boundingRect pattern
        color_detector.py uses on an HSV mask's contour."""
        contour = polygon.astype(np.int32).reshape(-1, 1, 2)
        moments = cv2.moments(contour)
        if moments['m00'] == 0:
            return None
        u = int(moments['m10'] / moments['m00'])
        v = int(moments['m01'] / moments['m00'])
        _, _, bbox_w, bbox_h = cv2.boundingRect(contour)
        return u, v, bbox_w, bbox_h, contour

    def _detect_instances(self, bgr, depth):
        confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        result = self.model.predict(bgr, conf=confidence_threshold, verbose=False)[0]

        # Publish regardless of whether anything was found, so "the topic
        # goes quiet" isn't confusable with "nothing detected" in the GUI.
        annotated = result.plot()  # BGR uint8, boxes+masks+labels+scores drawn
        debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        debug_msg.header.stamp = self.get_clock().now().to_msg()
        if self.camera_info is not None:
            debug_msg.header.frame_id = self.camera_info.header.frame_id
        self.debug_image_pub.publish(debug_msg)

        # Live OpenCV window -- real-time, no RViz/rqt/topic viewer needed.
        # waitKey(1) is required for the window to actually repaint each
        # call; it also pumps the 'q'-to-close check below.
        cv2.imshow('YOLO debug', annotated)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyWindow('YOLO debug')

        if result.masks is None:
            return []

        # (match_score, u, v, bbox_w_px, bbox_h_px, contour); match_score is
        # the detection's confidence if it matches target_class (or
        # target_class is empty), else -1 so it can never win the "best"
        # comparison below but still gets reported as an obstacle.
        candidates = []
        polygons = result.masks.xy
        classes = result.boxes.cls.cpu().numpy()
        confidences = result.boxes.conf.cpu().numpy()
        for polygon, cls_id, confidence in zip(polygons, classes, confidences):
            if polygon.shape[0] < 3:
                continue
            footprint = self._instance_footprint(polygon)
            if footprint is None:
                continue
            u, v, bbox_w_px, bbox_h_px, contour = footprint
            label = self.class_names.get(int(cls_id), str(int(cls_id))).lower()
            is_candidate = (not self.target_class) or (self.target_class in label)
            match_score = float(confidence) if is_candidate else -1.0
            candidates.append((match_score, u, v, bbox_w_px, bbox_h_px, contour))

        detections = []
        best_index = None
        for i, (score, u, v, bbox_w_px, bbox_h_px, _contour) in enumerate(candidates):
            detections.append(Detection2D(u=u, v=v, bbox_w_px=bbox_w_px, bbox_h_px=bbox_h_px))
            if score >= 0.0 and (best_index is None or score > candidates[best_index][0]):
                best_index = i

        if best_index is not None:
            detections[best_index].is_target = True
            mask = np.zeros(bgr.shape[:2], dtype=np.uint8)
            cv2.drawContours(mask, [candidates[best_index][5]], -1, 255, thickness=cv2.FILLED)
            detections[best_index].mask = mask

        return detections


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
