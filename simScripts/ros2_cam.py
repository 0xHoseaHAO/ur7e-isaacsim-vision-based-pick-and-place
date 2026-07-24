import omni.graph.core as og
from isaacsim.core.nodes.scripts.utils import set_target_prims


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
        camera_path: str,
        topic_name="/camera/camera_info",
        frame_id="camera_link",
        width=1280,
        height=720,
    ):
        """Publish intrinsics for the same render product as the RGB-D images."""
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("Tick", "omni.graph.action.OnPlaybackTick"),
                    (
                        "CreateRenderProduct",
                        "isaacsim.core.nodes.IsaacCreateRenderProduct",
                    ),
                    (
                        "CameraInfo",
                        "isaacsim.ros2.bridge.ROS2CameraInfoHelper",
                    ),
                ],
                keys.CONNECT: [
                    ("Tick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                    ("CreateRenderProduct.outputs:execOut", "CameraInfo.inputs:execIn"),
                    (
                        "CreateRenderProduct.outputs:renderProductPath",
                        "CameraInfo.inputs:renderProductPath",
                    ),
                ],
                keys.SET_VALUES: [
                    ("CreateRenderProduct.inputs:cameraPrim", camera_path),
                    ("CreateRenderProduct.inputs:width", width),
                    ("CreateRenderProduct.inputs:height", height),
                    ("CameraInfo.inputs:topicName", topic_name),
                    ("CameraInfo.inputs:frameId", frame_id),
                ],
            },
        )

    def publish_tf(self, graph_path: str, camera_path: str):
        """Publish the camera's live World transform on the standard /tf topic."""
        keys = og.Controller.Keys
        og.Controller.edit(
            {
                "graph_path": graph_path,
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
            },
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("SimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("ComputeTF", "isaacsim.core.nodes.IsaacComputeTransformTree"),
                    ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ],
                keys.SET_VALUES: [("PublishTF.inputs:topicName", "/tf")],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "ComputeTF.inputs:execIn"),
                    ("ComputeTF.outputs:execOut", "PublishTF.inputs:execIn"),
                    ("ComputeTF.outputs:parentFrames", "PublishTF.inputs:parentFrames"),
                    ("ComputeTF.outputs:childFrames", "PublishTF.inputs:childFrames"),
                    ("ComputeTF.outputs:translations", "PublishTF.inputs:translations"),
                    ("ComputeTF.outputs:orientations", "PublishTF.inputs:orientations"),
                    ("SimulationTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                ],
            },
        )
        set_target_prims(
            primPath=f"{graph_path}/ComputeTF",
            inputName="inputs:targetPrims",
            targetPrimPaths=[camera_path],
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
