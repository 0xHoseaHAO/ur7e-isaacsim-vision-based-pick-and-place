"""ROS 2 RGB-D HSV segmentation node."""

import rclpy
from rcl_interfaces.msg import SetParametersResult

from .color_detector import COLOR_RANGES, detect_color
from .perception_base import Detection2D, RGBDPerceptionNode


class SegmentationNode(RGBDPerceptionNode):
    """Detect a selected cube colour and recover its camera-frame 3D position."""

    def __init__(self) -> None:
        super().__init__('vision_segmentation')
        self.declare_parameter('target_color', 'red')
        self.declare_parameter('min_area', 100.0)

        self.target_color = str(self.get_parameter('target_color').value).lower()
        self.add_on_set_parameters_callback(self._on_parameter_change)
        self.get_logger().info(f'Looking for {self.target_color} cubes.')

    def _on_parameter_change(self, parameters):
        for parameter in parameters:
            if parameter.name == 'target_color':
                colour = str(parameter.value).lower()
                if colour not in COLOR_RANGES:
                    return SetParametersResult(
                        successful=False,
                        reason=f'target_color must be one of {sorted(COLOR_RANGES)}',
                    )
                self.target_color = colour
                self.get_logger().info(f'Target colour changed to {colour}')
        return SetParametersResult(successful=True)

    def _detect_instances(self, bgr, depth):
        # Scan every known colour, not just the target: pick_place_node
        # needs every other object's position (and size) too, to register
        # them as collision obstacles while it goes for the target one.
        min_area = float(self.get_parameter('min_area').value)
        detections = []
        for color in COLOR_RANGES:
            result = detect_color(bgr, color, min_area)
            if result is None:
                continue
            u, v, mask, (bbox_w_px, bbox_h_px) = result
            is_target = color == self.target_color
            detections.append(Detection2D(
                u=u,
                v=v,
                bbox_w_px=bbox_w_px,
                bbox_h_px=bbox_h_px,
                is_target=is_target,
                mask=mask if is_target else None,
            ))
        return detections


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
