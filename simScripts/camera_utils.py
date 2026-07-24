"""
camera_utils.py

Camera utilities for Isaac Sim 6.0.1
"""

import math

from pxr import Gf, UsdGeom
import omni.usd
import omni.kit.commands


class CameraUtils:

    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()

    def create_camera(
        self,
        name="MainCamera",
        position=(1.2, -1.2, 1.2),
        rotation=(55.0, 0.0, 45.0),
        parent_path="/World",
        horizontal_aperture=20.955,
        resolution=(1280, 720),
        horizontal_fov_deg=120.0,
        clipping_range=(0.05, 5.0),
    ):
        """
        Create a camera, optionally parented under an existing prim.

        Parenting the camera under a robot link (e.g. the end-effector's
        ``tool0`` frame) makes it an eye-in-hand camera: its world pose then
        follows that link's motion automatically through the USD xform
        hierarchy, with no extra TF wiring needed.

        ``horizontal_fov_deg`` drives the focal length so the field of view
        is specified directly instead of guessing a focal length. The
        default ``clipping_range`` is tuned for close-range tabletop
        manipulation, not a general scene: a far plane of 1000m makes every
        pixel that misses geometry report a depth wildly larger than the
        ~0.1-1m table distances, which crushes all the useful near-field
        depth into a couple of shades once linearly displayed.

        ``vertical_aperture`` is derived from ``resolution`` (matching the
        render product's width/height, e.g. what ros2_cam.py publishes at)
        rather than left at a fixed default: an aperture whose aspect ratio
        doesn't match the actual render resolution makes Kit's renderer
        crop or stretch one axis to reconcile them, so ``horizontal_fov_deg``
        stops reliably predicting what's actually in frame -- typically
        showing up as the vertical extent of the scene (e.g. a table's near/
        far edge for a top-down eye-in-hand camera) being narrower than
        expected regardless of how wide horizontal_fov_deg is set.

        Returns
        -------
        str
            Camera prim path.
        """

        camera_path = f"{parent_path}/{name}"

        #
        # Delete old camera
        #
        old = self.stage.GetPrimAtPath(camera_path)

        if old.IsValid():
            omni.kit.commands.execute(
                "DeletePrims",
                paths=[camera_path],
            )

        #
        # Create camera
        #
        camera = UsdGeom.Camera.Define(
            self.stage,
            camera_path,
        )

        xform = UsdGeom.XformCommonAPI(camera.GetPrim())

        xform.SetTranslate(
            Gf.Vec3d(
                float(position[0]),
                float(position[1]),
                float(position[2]),
            )
        )

        xform.SetRotate(
            Gf.Vec3f(
                float(rotation[0]),
                float(rotation[1]),
                float(rotation[2]),
            ),
            UsdGeom.XformCommonAPI.RotationOrderXYZ,
        )

        #
        # Camera parameters
        #
        width, height = resolution
        vertical_aperture = horizontal_aperture * (float(height) / float(width))
        focal_length = horizontal_aperture / (
            2.0 * math.tan(math.radians(horizontal_fov_deg) / 2.0)
        )
        vertical_fov_deg = 2.0 * math.degrees(
            math.atan(vertical_aperture / (2.0 * focal_length))
        )
        camera.GetHorizontalApertureAttr().Set(horizontal_aperture)
        camera.GetVerticalApertureAttr().Set(vertical_aperture)
        camera.GetFocalLengthAttr().Set(focal_length)
        camera.GetClippingRangeAttr().Set(
            Gf.Vec2f(float(clipping_range[0]), float(clipping_range[1]))
        )

        print("=" * 60)
        print("Camera created")
        print(f"Path      : {camera_path}")
        print(f"Position  : {position}")
        print(f"Rotation  : {rotation}")
        print(f"Resolution: {width}x{height}")
        print(
            f"FOV       : {horizontal_fov_deg:.1f} deg H / {vertical_fov_deg:.1f} deg V "
            f"(focal length {focal_length:.2f}mm)"
        )
        print(f"Clip range: {clipping_range}")
        print("=" * 60)

        return camera_path