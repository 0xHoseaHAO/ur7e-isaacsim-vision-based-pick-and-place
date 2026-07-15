import usdrt
import omni.graph.core as og


def create_moveit_action_graph(
    graph_path="/ROS2/MoveIt",
    robot_prim="/World/UR7e/Geometry/world/base_link",
    joint_state_topic="/joint_states",
    command_topic="/joint_command",
):

    keys = og.Controller.Keys

    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
        },
        {

            keys.CREATE_NODES: [

                ("Tick",
                 "omni.graph.action.OnPlaybackTick"),

                ("Context",
                 "isaacsim.ros2.bridge.ROS2Context"),

                ("ReadSimTime",
                 "isaacsim.core.nodes.IsaacReadSimulationTime"),

                ("ReadJointState",
                 "isaacsim.sensors.physics.IsaacReadJointState"),

                ("PublishJointState",
                 "isaacsim.ros2.bridge.ROS2PublishJointState"),

                ("SubscribeJointState",
                 "isaacsim.ros2.bridge.ROS2SubscribeJointState"),

                ("Controller",
                 "isaacsim.core.nodes.IsaacArticulationController"),

                ("PublishClock",
                 "isaacsim.ros2.bridge.ROS2PublishClock"),

            ],

            keys.CONNECT: [

                #
                # Publish Joint State
                #

                ("Tick.outputs:tick",
                 "ReadJointState.inputs:execIn"),

                ("ReadJointState.outputs:execOut",
                 "PublishJointState.inputs:execIn"),

                ("Context.outputs:context",
                 "PublishJointState.inputs:context"),

                ("ReadJointState.outputs:jointNames",
                 "PublishJointState.inputs:jointNames"),

                ("ReadJointState.outputs:jointPositions",
                 "PublishJointState.inputs:jointPositions"),

                ("ReadJointState.outputs:jointVelocities",
                 "PublishJointState.inputs:jointVelocities"),

                ("ReadJointState.outputs:jointEfforts",
                 "PublishJointState.inputs:jointEfforts"),

                ("ReadJointState.outputs:jointDofTypes",
                 "PublishJointState.inputs:jointDofTypes"),

                ("ReadJointState.outputs:stageMetersPerUnit",
                 "PublishJointState.inputs:stageMetersPerUnit"),

                ("ReadJointState.outputs:sensorTime",
                 "PublishJointState.inputs:sensorTime"),

                #
                # Subscribe Command
                #

                ("Tick.outputs:tick",
                 "SubscribeJointState.inputs:execIn"),

                ("Tick.outputs:tick",
                 "Controller.inputs:execIn"),

                ("Context.outputs:context",
                 "SubscribeJointState.inputs:context"),

                ("SubscribeJointState.outputs:jointNames",
                 "Controller.inputs:jointNames"),

                ("SubscribeJointState.outputs:positionCommand",
                 "Controller.inputs:positionCommand"),

                ("SubscribeJointState.outputs:velocityCommand",
                 "Controller.inputs:velocityCommand"),

                ("SubscribeJointState.outputs:effortCommand",
                 "Controller.inputs:effortCommand"),

                #
                # Clock
                #

                ("Tick.outputs:tick",
                 "PublishClock.inputs:execIn"),

                ("Context.outputs:context",
                 "PublishClock.inputs:context"),

                ("ReadSimTime.outputs:simulationTime",
                 "PublishClock.inputs:timeStamp"),

            ],

            keys.SET_VALUES: [

                (
                    "ReadJointState.inputs:prim",
                    [usdrt.Sdf.Path(robot_prim)],
                ),

                (
                    "Controller.inputs:robotPath",
                    robot_prim,
                ),

                (
                    "PublishJointState.inputs:topicName",
                    joint_state_topic,
                ),

                (
                    "SubscribeJointState.inputs:topicName",
                    command_topic,
                ),

            ],

        },
    )

    print("=" * 60)
    print("MoveIt Action Graph Created")
    print("=" * 60)