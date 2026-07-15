import omni.graph.core as og


class ROS2CameraPublisher:

    def __init__(self):
        pass

    def publish_rgb(
        self,
        graph_path: str,
        camera_path: str,
        topic_name="/camera/color/image_raw",
        frame_id="camera_link",
        width=1280,
        height=720,
    ):
        self._create_graph(
            graph_path,
            camera_path,
            topic_name,
            frame_id,
            width,
            height,
            sensor_type="rgb",
        )

    def publish_depth(
        self,
        graph_path: str,
        camera_path: str,
        topic_name="/camera/depth/image_raw",
        frame_id="camera_link",
        width=1280,
        height=720,
    ):
        self._create_graph(
            graph_path,
            camera_path,
            topic_name,
            frame_id,
            width,
            height,
            sensor_type="depth",
        )

    def publish_camera_info(
        self,
        graph_path: str,
        topic_name="/camera/camera_info",
        frame_id="camera_link",
    ):

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    (
                        "CameraInfo",
                        "isaacsim.ros2.bridge.ROS2CameraInfoHelper",
                    )
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("CameraInfo.inputs:topicName", topic_name),
                    ("CameraInfo.inputs:frameId", frame_id),
                ],
            },
        )

    def _create_graph(
        self,
        graph_path,
        camera_path,
        topic_name,
        frame_id,
        width,
        height,
        sensor_type,
    ):

        keys = og.Controller.Keys

        og.Controller.edit(
            {
                "graph_path": graph_path,
                "evaluator_name": "execution",
            },
            {
                keys.CREATE_NODES: [

                    (
                        "Tick",
                        "omni.graph.action.OnPlaybackTick",
                    ),

                    (
                        "CreateRenderProduct",
                        "isaacsim.core.nodes.IsaacCreateRenderProduct",
                    ),

                    (
                        "CameraHelper",
                        "isaacsim.ros2.bridge.ROS2CameraHelper",
                    ),

                ],

                keys.CONNECT: [

                    (
                        "Tick.outputs:tick",
                        "CreateRenderProduct.inputs:execIn",
                    ),

                    (
                        "CreateRenderProduct.outputs:execOut",
                        "CameraHelper.inputs:execIn",
                    ),

                    (
                        "CreateRenderProduct.outputs:renderProductPath",
                        "CameraHelper.inputs:renderProductPath",
                    ),

                ],

                keys.SET_VALUES: [

                    (
                        "CreateRenderProduct.inputs:cameraPrim",
                        camera_path,
                    ),

                    (
                        "CreateRenderProduct.inputs:width",
                        width,
                    ),

                    (
                        "CreateRenderProduct.inputs:height",
                        height,
                    ),

                    (
                        "CameraHelper.inputs:topicName",
                        topic_name,
                    ),

                    (
                        "CameraHelper.inputs:frameId",
                        frame_id,
                    ),

                    (
                        "CameraHelper.inputs:type",
                        sensor_type,
                    ),

                ],
            },
        )