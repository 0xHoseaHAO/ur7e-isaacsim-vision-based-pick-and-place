"""
bringup.py

One-command Isaac Sim bringup
Isaac Sim 6.0.1
"""
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": False,
    }
)

from pxr import Gf, Usd, UsdGeom, UsdPhysics

import omni.usd
import omni.kit.app

ext_manager = omni.kit.app.get_app().get_extension_manager()

ext_manager.set_extension_enabled_immediate(
    "isaacsim.ros2.bridge",
    True,
)

ext_manager.set_extension_enabled_immediate(
    "isaacsim.ros2.nodes",
    True,
)

print("ROS2 Bridge:",
      ext_manager.is_extension_enabled("isaacsim.ros2.bridge"))

print("ROS2 Nodes:",
      ext_manager.is_extension_enabled("isaacsim.ros2.nodes"))

# --------------------------------------------------------

from isaacsim.core.api import World

from scene_builder import SceneBuilder
from robot_loader import RobotLoader
from camera_utils import CameraUtils
from ros2_cam import ROS2CameraPublisher
from ros2_joint_command import create_joint_command_graph
from ros2_moveit import create_moveit_action_graph
from publish_joint_states import create_joint_state_publisher
from gripper_attach import GripperAttacher


def main():

    print("=" * 60)
    print("Creating Isaac Sim world...")
    print("=" * 60)

    world = World(stage_units_in_meters=1.0)

    #
    # Scene
    #
    SceneBuilder(world).build()

    #
    # Robot
    #
    loader = RobotLoader()

    robot = loader.load(
        urdf_path="./assets/robots/ur7e/ur7e.urdf",
        robot_name="UR7e",
        position=(0, 0, 0),
        fix_base=True,
    )

    print("Robot loaded.")

    #
    # Camera (eye-in-hand: mounted on the end-effector's tool0 frame so it
    # moves with the arm)
    #
    # The URDF's 'tool0'/'flange' links carry no geometry, so this importer
    # drops them and the deepest surviving wrist prim ends up named "wrist3"
    # instead. Try the ROS-Industrial name first, then fall back to whatever
    # this import actually produced.
    stage = omni.usd.get_context().get_stage()
    mount_link_candidates = ["tool0", "flange", "wrist3", "wrist_3_link"]
    prims_by_name = {}
    for p in stage.Traverse():
        if not str(p.GetPath()).startswith(robot.prim_path):
            continue
        prims_by_name.setdefault(p.GetName(), str(p.GetPath()))

    tool0_path = None
    for candidate in mount_link_candidates:
        if candidate in prims_by_name:
            tool0_path = prims_by_name[candidate]
            print(f"Mounting eye-in-hand camera on '{candidate}' at:", tool0_path)
            break

    if tool0_path is None:
        raise RuntimeError(
            f"Could not find any of {mount_link_candidates} under {robot.prim_path}; "
            "cannot mount eye-in-hand camera."
        )

    camera = CameraUtils()

    # gripper_reach no longer sizes anything visible (the gripper
    # placeholder mesh is gone -- see the gripper_tip anchor below), but
    # GripperAttacher still needs a numeric reach/attach_radius, so it
    # stays as a plain constant.
    gripper_reach = 0.16

    # NOTE: the prim name here must match the frame_id passed below.
    # ComputeTransformTree/PublishTF names the TF child frame after this
    # prim ("wrist_camera"); if the sensor messages used a different
    # frame_id (e.g. "WristCamera"/"wrist_camera" case mismatch), tf2
    # lookups against the pose's frame_id would never resolve.
    camera_name = "wrist_camera"
    # Shared with the ROS2 publishers below so the camera's aperture
    # (which determines its *actual* field of view) and the render
    # product it's actually rendered at can never drift apart -- see
    # create_camera's docstring for why a mismatch there silently crops
    # the frame regardless of horizontal_fov_deg.
    camera_resolution = (1280, 720)
    # With the gripper placeholder mesh removed, nothing sits coaxial with
    # tool0's forward axis anymore, so the camera no longer needs the local
    # X offset that used to push it out of the gripper's own view. Only a
    # small forward (+Z) offset remains, purely to clear the
    # wrist_3_link/flange geometry itself -- tune the magnitude once
    # running in Isaac Sim to match the mount.
    camera_forward_offset = 0.03
    camera_path = camera.create_camera(
        name=camera_name,
        parent_path=tool0_path,
        # tool0's +Z is "front" (see the flange-tool0 comment in the URDF);
        # rotation=(180,0,0) points the camera's view direction (-Z in its
        # own frame) down tool0's +Z, i.e. forward.
        position=(0.0, 0.0, camera_forward_offset),
        rotation=(180.0, 0.0, 0.0),
        resolution=camera_resolution,
        horizontal_fov_deg=120.0,
        clipping_range=(0.05, 3.0),
    )

    # Ground truth for "how far is the camera from the mount / where did it
    # actually land in world space" instead of guessing from local offsets.
    def _world_position(prim_path):
        prim = stage.GetPrimAtPath(prim_path)
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return tuple(matrix.ExtractTranslation())

    print(f"'{candidate}' world position:", _world_position(tool0_path))
    print(f"'{camera_name}' world position:", _world_position(camera_path))

    ros = ROS2CameraPublisher()

    ros.publish_rgb(
        "/ROS2/RGB",
        camera_path,
        frame_id=camera_name,
        width=camera_resolution[0],
        height=camera_resolution[1],
    )

    ros.publish_depth(
        "/ROS2/Depth",
        camera_path,
        frame_id=camera_name,
        width=camera_resolution[0],
        height=camera_resolution[1],
    )

    ros.publish_camera_info(
        "/ROS2/CameraInfo",
        camera_path,
        frame_id=camera_name,
        width=camera_resolution[0],
        height=camera_resolution[1],
    )

    ros.publish_tf(
        "/ROS2/CameraTF",
        camera_path,
    )

    #
    # Gripper tip (invisible anchor, no mesh): the UR7e model has no
    # gripper at all, and the real one referenced from ur10_rg2_ros-master
    # (robot_descriptions/description/rg2.urdf.xacro) is itself just a
    # fixed, non-articulated mesh -- there's no finger joint to actually
    # drive either way. Earlier this anchor also carried a visible
    # placeholder shape (a shaft + jaws block); that mesh has been removed,
    # but the anchor prim itself stays: gripper_attach.py still uses it as
    # its reach point (nearest-object search + attach-radius), and
    # pick_place_node's grasp_height_offset must be kept in sync with this
    # value by hand (it's a separate ROS2 parameter in a different
    # process, so it can't just share this variable the way the camera
    # mount offset above now does). `gripper_reach` out from the wrist
    # along its local +Z ("front" per the tool0 comment above -- but if the
    # mount fell back to 'wrist3' instead of a real 'tool0', that axis
    # convention isn't verified).
    #
    gripper_tip_path = f"{tool0_path}/gripper_tip"
    gripper_tip_prim = UsdGeom.Xform.Define(stage, gripper_tip_path)
    UsdGeom.XformCommonAPI(gripper_tip_prim.GetPrim()).SetTranslate(Gf.Vec3d(0.0, 0.0, gripper_reach))

    print(f"Gripper tip anchor (no visible mesh) mounted {gripper_reach:.3f}m out from '{candidate}'.")

    #
    # Joint State Publisher
    #
    create_joint_state_publisher(
        graph_path="/ROS2/JointState",
        robot_prim="/World/UR7e/Geometry/world/base_link",
        topic="/joint_states",
    )

    #
    # Initialize
    #
    world.reset()
    robot.initialize()

    print("=" * 60)
    print("Robot initialized")
    print("=" * 60)

    print("DOF:", robot.num_dof)
    print("Joint Names:", robot.dof_names)

    #
    # Table for the rods to rest on, within the arm's reach. Its collision
    # geometry for MoveIt/cuMotion isn't set up here at all -- that comes
    # entirely from segmentation_node's depth-based surface detector at
    # runtime, the same way it would for any other flat surface.
    #
    from isaacsim.core.utils.stage import add_reference_to_stage

    table_usd_path = Path("./assets/table/packing_table.usd").resolve()
    if not table_usd_path.exists():
        raise FileNotFoundError(table_usd_path)

    table_prim_path = "/World/Table"
    add_reference_to_stage(usd_path=str(table_usd_path), prim_path=table_prim_path)

    table_prim = stage.GetPrimAtPath(table_prim_path)
    UsdGeom.XformCommonAPI(table_prim).SetTranslate(Gf.Vec3d(0, -0.8, -0.9))

    # Read the asset's actual footprint back instead of guessing its size,
    # just to know where to place the rods (and the person, below) --
    # purely a scene-setup concern.
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
    )
    table_range = bbox_cache.ComputeWorldBound(table_prim).ComputeAlignedRange()
    table_min, table_max = table_range.GetMin(), table_range.GetMax()
    table_top_z = float(table_max[2])
    table_center_x = float((table_min[0] + table_max[0]) / 2.0)
    table_center_y = float((table_min[1] + table_max[1]) / 2.0)
    print(f"Table top surface z = {table_top_z:.3f}, centre = ({table_center_x:.3f}, {table_center_y:.3f})")

    #
    # Person figurine on the table -- gives the pretrained YOLO model
    # (target_class:=person) something in-vocabulary to actually detect,
    # unlike the synthetic colour rods.
    #
    person_usd_path = Path("./assets/male_adult/person.usd").resolve()
    if not person_usd_path.exists():
        raise FileNotFoundError(person_usd_path)

    person_prim_path = "/World/Person"
    add_reference_to_stage(usd_path=str(person_usd_path), prim_path=person_prim_path)

    person_prim = stage.GetPrimAtPath(person_prim_path)
    person_xform = UsdGeom.XformCommonAPI(person_prim)
    person_xform.SetTranslate(Gf.Vec3d(0, -0.5, 0.10))  # fixed for now -- tune by hand
    # XformCommonAPI.SetScale always writes a GfVec3f, but person.usd's
    # referenced root prim already authors xformOp:scale as double3 --
    # USD won't let a float3 value be written into an already-double3
    # attribute, so set it directly instead of through XformCommonAPI.
    person_scale_attr = person_prim.GetAttribute('xformOp:scale')
    if person_scale_attr.IsValid():
        person_scale_attr.Set(Gf.Vec3d(0.08, 0.08, 0.08))
    else:
        person_xform.SetScale(Gf.Vec3f(0.08, 0.08, 0.08))
    # Same double3-vs-float3 issue as the scale op above.
    person_rotate_attr = person_prim.GetAttribute('xformOp:rotateXYZ')
    if person_rotate_attr.IsValid():
        person_rotate_attr.Set(Gf.Vec3d(270.0, 0.0, 180.0))
    else:
        person_xform.SetRotate(Gf.Vec3f(270.0, 0.0, 180.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    # A referenced character asset like this carries no physics of its own
    # (no RigidBodyAPI, no collision on its many child meshes). Give its
    # root prim a rigid body (so GripperAttacher's FixedJoint below has a
    # valid target) plus gravity and a simple box collision proxy standing
    # in for its actual geometry -- exact per-mesh collision on an
    # arbitrary character rig is more than this needs; a rough box is
    # enough for it to rest on the table. Earlier this had gravity
    # *disabled* with no collider at all, so it wouldn't fall through the
    # table -- but that also meant nothing stopped it once the gripper let
    # go: with whatever velocity it had at that instant and nothing to
    # arrest it (no gravity to pull it back down, no collider to hit the
    # table), it just sailed off. A real collider + gravity fixes both.
    UsdPhysics.RigidBodyAPI.Apply(person_prim)
    UsdPhysics.MassAPI.Apply(person_prim).CreateMassAttr().Set(5.0)

    person_collider_path = f"{person_prim_path}/collision_proxy"
    person_collider = UsdGeom.Cube.Define(stage, person_collider_path)
    person_collider.GetSizeAttr().Set(1.0)
    # Non-uniform scale on the proxy itself, in the parent's pre-scale
    # local space -- this is a rough placeholder box; tune by hand (size
    # and SetTranslate below) to actually match the model once you can see
    # it posed/rotated in the viewport.
    UsdGeom.XformCommonAPI(person_collider.GetPrim()).SetScale(Gf.Vec3f(1.0, 1.0,1.0))
    UsdGeom.Imageable(person_collider.GetPrim()).MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(person_collider.GetPrim())

    #
    # Shared helper for the donut/ramp/bear placement below.
    #
    def _world_bounds(prim):
        cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
        )
        return cache.ComputeWorldBound(prim).ComputeAlignedRange()

    #
    # donut on the table -- another COCO-class object ("cup") for YOLO to
    # detect and for the gripper to pick, same treatment as the person
    # figurine above: reference the asset, normalize it to a sane
    # real-world size (donuts aren't authored to any particular scale), then
    # give it a rigid body + box collider so it can rest on the table and
    # be lifted.
    #
    # The reference is added to a *child* prim ("Asset") rather than to the
    # transformed prim itself: some referenced assets author a single
    # combined xformOp:transform matrix on their root instead of separate
    # translate/rotate/scale ops, and UsdGeomXformCommonAPI silently no-ops
    # (no error, transform just never changes) when asked to touch a prim
    # shaped like that -- see the teddy bear below, which hit exactly this.
    # Keeping our own transform on a fresh wrapper prim sidesteps it.
    #
    donut_usd_path = Path("./assets/donut/donut.usd").resolve()
    if not donut_usd_path.exists():
        raise FileNotFoundError(donut_usd_path)

    donut_prim_path = "/World/donut"
    donut_asset_path = f"{donut_prim_path}/Asset"
    donut_prim = UsdGeom.Xform.Define(stage, donut_prim_path).GetPrim()
    add_reference_to_stage(usd_path=str(donut_usd_path), prim_path=donut_asset_path)
    donut_asset_prim = stage.GetPrimAtPath(donut_asset_path)
    donut_xform = UsdGeom.XformCommonAPI(donut_prim)

    # Rotate first: XformCommonAPI always composes scale, then rotate, then
    # translate, regardless of the order these Set* calls are made in -- but
    # the *bounds* queries below reflect only whatever's been authored so
    # far. Measuring height/bottom before this rotation was in effect would
    # read the asset's pre-rotation (wrong-axis) extents, throwing off both
    # the scale factor and the table-surface offset below.
    donut_xform.SetRotate(Gf.Vec3f(0.0, 0.0, 0.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    donut_target_height = 0.08  # roughly a real donut, in metres -- tune by hand
    donut_native_range = _world_bounds(donut_asset_prim)
    donut_native_height = float(donut_native_range.GetMax()[2] - donut_native_range.GetMin()[2])
    donut_scale_factor = donut_target_height / donut_native_height if donut_native_height > 0 else 1.0
    donut_xform.SetScale(Gf.Vec3f(donut_scale_factor, donut_scale_factor, donut_scale_factor))

    donut_bottom_z = float(_world_bounds(donut_prim).GetMin()[2])
    donut_pos_x, donut_pos_y = table_center_x -0.3, table_center_y +0.15
    donut_xform.SetTranslate(Gf.Vec3d(donut_pos_x, donut_pos_y, table_top_z - donut_bottom_z + 0.01))
    donut_xform.SetRotate(Gf.Vec3f(0.0, 0.0, 270.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    UsdPhysics.RigidBodyAPI.Apply(donut_prim)
    UsdPhysics.MassAPI.Apply(donut_prim).CreateMassAttr().Set(0.3)

    donut_collider_path = f"{donut_prim_path}/collision_proxy"
    donut_collider = UsdGeom.Cube.Define(stage, donut_collider_path)
    donut_collider.GetSizeAttr().Set(1.0)
    # Collider size is authored in the donut prim's own pre-scale local
    # space, so divide the desired real-world box (roughly the donut's
    # footprint x height) by its scale factor to cancel that back out --
    # a rough placeholder box; tune by hand once visible in the viewport.
    UsdGeom.XformCommonAPI(donut_collider.GetPrim()).SetScale(
        Gf.Vec3f(
            (donut_target_height * 0.7) / donut_scale_factor,
            (donut_target_height * 0.7) / donut_scale_factor,
            donut_target_height / donut_scale_factor,
        )
    )
    UsdGeom.Imageable(donut_collider.GetPrim()).MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(donut_collider.GetPrim())

    #
    # Ramp for the teddy bear -- no ramp asset ships in assets/, so this
    # builds a simple inclined static prop from a scaled+rotated cube, with
    # a physics collider (no RigidBodyAPI, so it stays fixed in place like
    # the table). The bear below rests on this collider under gravity and
    # ends up tilted to the incline, the same way the rods/person rest on
    # the table's own collider.
    #
    ramp_prim_path = "/World/Ramp"
    ramp_size = Gf.Vec3f(0.28, 0.22, 0.02)  # up-slope length x width x thickness -- tune by hand
    ramp_tilt_deg = 45.0  # incline angle -- tune by hand

    ramp_prim = UsdGeom.Cube.Define(stage, ramp_prim_path)
    ramp_prim.GetSizeAttr().Set(1.0)
    ramp_xform = UsdGeom.XformCommonAPI(ramp_prim.GetPrim())
    ramp_xform.SetScale(ramp_size)
    ramp_xform.SetRotate(Gf.Vec3f(ramp_tilt_deg, 0.0, 180.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    ramp_pos_x, ramp_pos_y, ramp_pos_z = table_center_x + 0.15, table_center_y + 0.10, 0.10
    ramp_xform.SetTranslate(Gf.Vec3d(ramp_pos_x, ramp_pos_y, ramp_pos_z))

    # Bounds read back *after* the translate above (not before) so they
    # reflect where the ramp actually ended up -- used below to know how
    # high the bear needs to start to land on the incline.
    ramp_bounds = _world_bounds(ramp_prim.GetPrim())
    ramp_bottom_z = float(ramp_bounds.GetMin()[2])
    ramp_top_z = float(ramp_bounds.GetMax()[2])

    UsdPhysics.CollisionAPI.Apply(ramp_prim.GetPrim())

    #
    # Teddy bear -- another COCO class ("teddy bear") for YOLO. Scaled,
    # tilted to the ramp's incline angle, and dropped just above its high
    # end so gravity settles it resting against the slope rather than
    # standing upright on it.
    #
    # teddy_bear.usd is UE/FBX-sourced (see the OmniUe4Base/OmniUe4Function
    # MDL resolver warnings at startup) and authors a single combined
    # xformOp:transform matrix on its root prim -- XformCommonAPI silently
    # no-ops on that shape of prim, which is why it kept sitting at the
    # origin no matter what SetTranslate/SetRotate/SetScale was called
    # with. Referencing onto a child of a fresh wrapper Xform (same fix as
    # the donut above) avoids ever calling CommonAPI on the asset's own root.
    #
    bear_usd_path = Path("./assets/Teddy_Bear/teddy_bear.usd").resolve()
    if not bear_usd_path.exists():
        raise FileNotFoundError(bear_usd_path)

    bear_prim_path = "/World/TeddyBear"
    bear_asset_path = f"{bear_prim_path}/Asset"
    bear_prim = UsdGeom.Xform.Define(stage, bear_prim_path).GetPrim()
    add_reference_to_stage(usd_path=str(bear_usd_path), prim_path=bear_asset_path)
    bear_asset_prim = stage.GetPrimAtPath(bear_asset_path)
    bear_xform = UsdGeom.XformCommonAPI(bear_prim)

    bear_target_height = 0.10  # roughly a small teddy bear, in metres -- tune by hand
    bear_native_range = _world_bounds(bear_asset_prim)
    bear_native_height = float(bear_native_range.GetMax()[2] - bear_native_range.GetMin()[2])
    bear_scale_factor = bear_target_height / bear_native_height if bear_native_height > 0 else 1.0
    bear_xform.SetScale(Gf.Vec3f(bear_scale_factor, bear_scale_factor, bear_scale_factor))
    bear_xform.SetRotate(Gf.Vec3f(-45.0, 0.0, 180.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    bear_bottom_z = float(_world_bounds(bear_prim).GetMin()[2])
    # Start it just above the ramp's high point so it drops onto the
    # incline under gravity instead of spawning inside the collider.
    bear_xform.SetTranslate(
        Gf.Vec3d(ramp_pos_x, ramp_pos_y, ramp_top_z - bear_bottom_z + 0.03)
    )

    UsdPhysics.RigidBodyAPI.Apply(bear_prim)
    UsdPhysics.MassAPI.Apply(bear_prim).CreateMassAttr().Set(0.15)

    bear_collider_path = f"{bear_prim_path}/collision_proxy"
    bear_collider = UsdGeom.Cube.Define(stage, bear_collider_path)
    bear_collider.GetSizeAttr().Set(1.0)
    UsdGeom.XformCommonAPI(bear_collider.GetPrim()).SetScale(
        Gf.Vec3f(
            (bear_target_height * 0.6) / bear_scale_factor,
            (bear_target_height * 0.5) / bear_scale_factor,
            bear_target_height / bear_scale_factor,
        )
    )
    UsdGeom.Imageable(bear_collider.GetPrim()).MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(bear_collider.GetPrim())

    #
    # Coloured wooden rods to pick (elongated cuboids so grasp orientation
    # matters, matching the real target objects), resting on the table top
    #
    from isaacsim.core.api.objects import DynamicCuboid

    rod_scale = np.array([0.20, 0.05, 0.05])
    rod_z = table_top_z + rod_scale[2] / 2.0
    rods = [
      #  ("RedRod", (table_center_x, table_center_y, rod_z), (1.0, 0.0, 0.0)),
        #("YellowRod", (table_center_x + 0.15, table_center_y - 0.10, rod_z), (1.0, 1.0, 0.0)),
       # ("BlueRod", (table_center_x - 0.15, table_center_y + 0.10, rod_z), (0.0, 0.0, 1.0)),
    ]
    for name, position, color in rods:
        world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/{name}",
                name=name.lower(),
                position=np.array(position),
                scale=rod_scale,
                color=np.array(color),
            )
        )

    #
    # Gripper: the UR7e model has no fingers, so this stands in for one by
    # rigidly attaching (via a runtime FixedJoint) whichever rod is
    # currently nearest the gripper tip when the gripper closes, and
    # detaching it again on open -- see gripper_attach.py for why this
    # replaced Isaac Sim's official Surface Gripper plugin. Poll-driven
    # off a state file rather than an rclpy subscription -- running rclpy
    # inside Isaac Sim's own process kept crashing (mixed bundled/system
    # rclpy builds), so this needs no ROS2 context here at all.
    #
    rod_paths = [f"/World/{name}" for name, _, _ in rods]
    attachable_paths = rod_paths + [person_prim_path, donut_prim_path, bear_prim_path]
    attacher = GripperAttacher(stage, tool0_path, gripper_tip_path, attachable_paths, attach_radius=gripper_reach)

    #
    # MoveIt
    #
    create_moveit_action_graph(
        robot_prim="/World/UR7e/Geometry/world/base_link",
    )

    #
    # Joint Command
    #
    create_joint_command_graph(
        robot_prim="/World/UR7e/Geometry/world/base_link",
        topic="/joint_command",
    )

    print("=" * 60)
    print("Simulation started")
    print("=" * 60)

    frame = 0

    while simulation_app.is_running():

        world.step(render=True)
        attacher.update()

        frame += 1

       # if frame % 120 == 0:
            #print("Frame:", frame)
            #print(robot.get_joint_positions())

    simulation_app.close()


if __name__ == "__main__":
    main()
