"""
camera_utils.py

Camera utilities for Isaac Sim 6.0.1
"""

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
    ):
        """
        Create a camera.

        Returns
        -------
        str
            Camera prim path.
        """

        camera_path = f"/World/{name}"

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
        camera.GetHorizontalApertureAttr().Set(20.955)
        camera.GetVerticalApertureAttr().Set(15.2908)
        camera.GetFocalLengthAttr().Set(24.0)
        camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1000.0))

        print("=" * 60)
        print("Camera created")
        print(f"Path      : {camera_path}")
        print(f"Position  : {position}")
        print(f"Rotation  : {rotation}")
        print("=" * 60)

        return camera_path