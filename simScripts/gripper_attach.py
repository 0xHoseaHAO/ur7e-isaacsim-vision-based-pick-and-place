"""gripper_attach.py

Direct rigid attach: on gripper-close, find whichever tracked object is
currently nearest the gripper tip (within `attach_radius`) and weld it
there with a runtime UsdPhysics.FixedJoint, removed again on release.

This replaces an earlier attempt at Isaac Sim's official Surface Gripper
plugin (isaacsim.robot.surface_gripper), whose D6 attachment joint never
got its axis/frame conventions verified against this URDF -- it kept
reporting "Closing" without ever finishing an attach. This module instead
just snaps a rigid, unbreakable joint onto whichever candidate object is
closest, the moment the gripper is commanded shut, rather than sensing a
grasp through the plugin's own proximity/force logic. The joint targets
tool0 directly (excluded from the robot's articulation, same as any
robot-link-to-free-body attach) with its frame computed from the current
poses of both bodies, so attaching never snaps the object to a new
position -- it's picked up exactly where it already is.

Still file-polling instead of an rclpy subscription for the same reason
as before: running rclpy inside Isaac Sim's own process repeatedly hit
native crashes (mixing Isaac Sim's bundled rclpy build with the system
one). pick_place_node (or gripper_bridge.py, if pick_place_node runs
somewhere that doesn't share Isaac Sim's /tmp -- e.g. inside the cuMotion
Docker container) writes '0'/'1' to GRIPPER_STATE_FILE; this module just
polls it.
"""

from pxr import Gf, Usd, UsdGeom, UsdPhysics

# Must match GRIPPER_STATE_FILE in vision_pick/vision_pick/pick_place_node.py
# and simScripts/gripper_bridge.py.
GRIPPER_STATE_FILE = '/tmp/vision_pick_gripper_state'

_ATTACH_JOINT_PATH = '/World/_gripper_attach_joint'


def _world_transform(prim):
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


class GripperAttacher:
    """Polls GRIPPER_STATE_FILE once per simulation step; on a close
    transition, rigidly attaches whichever tracked object is currently
    nearest `gripper_tip_path` (within attach_radius) to `tool0_path` via
    a runtime FixedJoint. On an open transition, removes that joint."""

    def __init__(self, stage, tool0_path, gripper_tip_path, object_paths, attach_radius=0.08):
        self.stage = stage
        self.tool0_path = tool0_path
        self.gripper_tip_path = gripper_tip_path
        self.object_paths = list(object_paths)
        self.attach_radius = attach_radius
        self._last_state = False
        self._attached_path = None
        self._update_count = 0
        # A stale '1' left over from a previous run would otherwise read as
        # an immediate (bogus) close the moment this starts polling, before
        # the arm has moved anywhere near an object.
        try:
            with open(GRIPPER_STATE_FILE, 'w') as state_file:
                state_file.write('0')
        except OSError as exc:
            print(f'[gripper_attach] could not reset state file: {exc}', flush=True)
        print(f'[gripper_attach] watching {GRIPPER_STATE_FILE}', flush=True)

    def _read_close_state(self):
        try:
            with open(GRIPPER_STATE_FILE) as state_file:
                return state_file.read().strip() == '1'
        except FileNotFoundError:
            return False
        except OSError as exc:
            print(f'[gripper_attach] could not read state file: {exc}', flush=True)
            return self._last_state

    def _nearest_object(self):
        tip_pos = _world_transform(self.stage.GetPrimAtPath(self.gripper_tip_path)).ExtractTranslation()
        nearest_path, nearest_dist = None, self.attach_radius
        # Track the single closest candidate even if it's outside
        # attach_radius, purely so a failed close can report how far off it
        # actually was instead of just "nothing" -- distance here is to
        # each prim's own pivot, which for a referenced asset (e.g. a
        # character rig) may sit far from its visible geometry.
        closest_path, closest_dist = None, None
        for path in self.object_paths:
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue
            pos = _world_transform(prim).ExtractTranslation()
            dist = (pos - tip_pos).GetLength()
            if closest_dist is None or dist < closest_dist:
                closest_path, closest_dist = path, dist
            if dist <= nearest_dist:
                nearest_path, nearest_dist = path, dist
        if nearest_path is None and closest_path is not None:
            print(
                f'[gripper_attach] close -> nothing within attach_radius='
                f'{self.attach_radius:.3f}m (closest was {closest_path} at '
                f'{closest_dist:.3f}m)',
                flush=True,
            )
        return nearest_path

    def _attach(self, object_path):
        tool0_to_world = _world_transform(self.stage.GetPrimAtPath(self.tool0_path))
        object_to_world = _world_transform(self.stage.GetPrimAtPath(object_path))
        # Joint frame anchored at tool0's own origin (identity on the body0
        # side); on the body1 side it's wherever that same world-space
        # frame currently lands in the object's own local frame -- so the
        # joint starts at zero relative-pose error (holds the object
        # exactly where it already is) instead of snapping it somewhere
        # else.
        tool0_in_object = tool0_to_world * object_to_world.GetInverse()
        pos1 = tool0_in_object.ExtractTranslation()
        quat1 = tool0_in_object.ExtractRotationQuat()
        imag1 = quat1.GetImaginary()

        joint = UsdPhysics.FixedJoint.Define(self.stage, _ATTACH_JOINT_PATH)
        joint.CreateBody0Rel().SetTargets([self.tool0_path])
        joint.CreateBody1Rel().SetTargets([object_path])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(float(pos1[0]), float(pos1[1]), float(pos1[2])))
        joint.CreateLocalRot1Attr().Set(
            Gf.Quatf(float(quat1.GetReal()), float(imag1[0]), float(imag1[1]), float(imag1[2]))
        )
        joint.CreateBreakForceAttr().Set(3.4028235e38)
        joint.CreateBreakTorqueAttr().Set(3.4028235e38)
        # tool0 is part of the robot's reduced-coordinate articulation;
        # excluding this joint keeps it solved as a regular maximal-
        # coordinate joint against the object instead of being folded into
        # (and fighting) the articulation solve.
        joint.CreateExcludeFromArticulationAttr().Set(True)
        self._attached_path = object_path
        print(f'[gripper_attach] close -> attached {object_path}', flush=True)

    def _detach(self):
        if self._attached_path is not None:
            self.stage.RemovePrim(_ATTACH_JOINT_PATH)
            print(f'[gripper_attach] open -> released {self._attached_path}', flush=True)
        else:
            print('[gripper_attach] open -> nothing was attached', flush=True)
        self._attached_path = None

    def update(self):
        """Call once per simulation step."""
        state = self._read_close_state()

        self._update_count += 1
        if self._update_count % 300 == 0:
            print(
                f'[gripper_attach] heartbeat: state_file says '
                f'{"close" if state else "open"}, attached={self._attached_path}',
                flush=True,
            )

        if state == self._last_state:
            return
        self._last_state = state

        if state:
            nearest = self._nearest_object()  # prints its own diagnostic if nothing qualifies
            if nearest is not None:
                self._attach(nearest)
        else:
            self._detach()
