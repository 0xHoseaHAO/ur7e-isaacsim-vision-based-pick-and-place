import usdrt
import omni.graph.core as og


def create_joint_state_publisher(
    graph_path="/ROS2/JointState",
    robot_prim="/World/UR7e/Geometry/world/base_link",
    topic="/joint_states",
):
    keys = og.Controller.Keys

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],

            keys.SET_VALUES: [
                (
                    "PublishJointState.inputs:targetPrim",
                    [usdrt.Sdf.Path(robot_prim)],
                ),
                (
                    "PublishJointState.inputs:topicName",
                    topic,
                ),
            ],

            keys.CONNECT: [
                (
                    "Tick.outputs:tick",
                    "PublishJointState.inputs:execIn",
                ),
                (
                    "SimTime.outputs:simulationTime",
                    "PublishJointState.inputs:timeStamp",
                ),
            ],
        },
    )

    print("=" * 60)
    print("ROS2 JointState Publisher Created")
    print(f"Robot : {robot_prim}")
    print(f"Topic : {topic}")
    print(f"Graph : {graph_path}")
    print("=" * 60)