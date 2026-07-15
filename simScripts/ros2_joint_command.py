import omni.graph.core as og

import omni.graph.core as og


def create_joint_command_graph(
    robot_prim="/World/UR7e/Geometry/world/base_link",
    topic="/joint_command",
):

    graph_path = "/ROS2/JointCommand"

    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
        },
        {
            "create_nodes": [

                (
                    "OnPlaybackTick",
                    "omni.graph.action.OnPlaybackTick",
                ),

                (
                    "JointStateSubscriber",
                    "isaacsim.ros2.bridge.ROS2SubscribeJointState",
                ),

                (
                    "ArticulationController",
                    "isaacsim.core.nodes.IsaacArticulationController",
                ),
            ],

            "connect": [

                # execution
                (
                    "OnPlaybackTick.outputs:tick",
                    "JointStateSubscriber.inputs:execIn",
                ),

                (
                    "JointStateSubscriber.outputs:execOut",
                    "ArticulationController.inputs:execIn",
                ),

                # joint names
                (
                    "JointStateSubscriber.outputs:jointNames",
                    "ArticulationController.inputs:jointNames",
                ),

                # positions
                (
                    "JointStateSubscriber.outputs:positionCommand",
                    "ArticulationController.inputs:positionCommand",
                ),

                # velocity
                (
                    "JointStateSubscriber.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),

                # effort
                (
                    "JointStateSubscriber.outputs:effortCommand",
                    "ArticulationController.inputs:effortCommand",
                ),

            ],

            "set_values": [

                (
                    "JointStateSubscriber.inputs:topicName",
                    topic,
                ),

                (
                    "ArticulationController.inputs:targetPrim",
                    robot_prim,
                ),

            ],
        },
    )

    print("=" * 60)
    print("Joint Command Graph Created")
    print("=" * 60)