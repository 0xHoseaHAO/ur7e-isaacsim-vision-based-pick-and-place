"""
robot_loader.py

Generic URDF loader for Isaac Sim 6.0.1
"""

from pathlib import Path

import omni.usd
import omni.kit.commands

from pxr import Gf, UsdGeom, UsdPhysics

from isaacsim.asset.importer.urdf import (
    URDFImporter,
    URDFImporterConfig,
)

from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation


class RobotLoader:

    def __init__(self):
        pass

    def load(
        self,
        urdf_path: str,
        robot_name: str = "Robot",
        position=(0.0, 0.0, 0.0),
        fix_base=True,
        merge_mesh=False,
    ):

        urdf_path = Path(urdf_path).resolve()

        if not urdf_path.exists():
            raise FileNotFoundError(urdf_path)

        #
        # URDF Import Config
        #
        config = URDFImporterConfig()

        config.urdf_path = str(urdf_path)
        config.fix_base = fix_base
        config.merge_mesh = merge_mesh

        config.import_visual_meshes = True
        config.import_collision_meshes = True
        config.bundle_subfolders = True
        config.force_override = True
        config.import_joints = True
        config.replace_inertial_meshes = True
        config.override_joint_stiffness = 10000.0
        config.override_joint_damping = 1000.0
        importer = URDFImporter(config)
        print(dir(importer))
        print(type(importer))
        config = URDFImporterConfig()
        print(config.joint_drive_type)
        print(config.joint_target_type)
        print(type(config.joint_drive_type))
        print(type(config.joint_target_type))   
        print(dir(config))
        print("=" * 60)
        print("Importing URDF...")
        print("=" * 60)

        usd_path = importer.import_urdf()

        print("Generated USD:")
        print(usd_path)

        #
        # Spawn
        #
        stage = omni.usd.get_context().get_stage()

        prim_path = f"/World/{robot_name}"

        if stage.GetPrimAtPath(prim_path).IsValid():
            omni.kit.commands.execute(
                "DeletePrim",
                path=prim_path,
            )

        add_reference_to_stage(
            usd_path=usd_path,
            prim_path=prim_path,
        )

        #
        # Move Robot
        #
        prim = stage.GetPrimAtPath(prim_path)

        if not prim.IsValid():
            raise RuntimeError(f"Cannot find {prim_path}")

        xform = UsdGeom.XformCommonAPI(prim)

        xform.SetTranslate(
            Gf.Vec3d(
                position[0],
                position[1],
                position[2],
            )
        )

        print("=" * 60)
        print("Robot Children")

        for child in prim.GetChildren():
            print(" -", child.GetName())

        #
        # Search Articulation Root
        #
        print("=" * 60)
        print("Searching Articulation Root...")

        articulation_path = None

        for p in stage.Traverse():

            if not str(p.GetPath()).startswith(prim_path):
                continue

            if p.HasAPI(UsdPhysics.ArticulationRootAPI):

                articulation_path = str(p.GetPath())

                print("Found:", articulation_path)

                break

        if articulation_path is None:
            raise RuntimeError(
                "No Articulation Root found!"
            )

        print("=" * 60)

        #
        # Print Drives
        #
        print("Drive APIs:")

        for p in stage.Traverse():

            if not str(p.GetPath()).startswith(prim_path):
                continue

            drive = UsdPhysics.DriveAPI.Get(p, "angular")

            if drive:

                print(
                    p.GetPath(),
                    " stiffness=",
                    drive.GetStiffnessAttr().Get(),
                    " damping=",
                    drive.GetDampingAttr().Get(),
                    " maxForce=",
                    drive.GetMaxForceAttr().Get(),
                )

        print("=" * 60)

        #
        # Create Articulation
        #
        robot = SingleArticulation(
            prim_path=articulation_path,
            name=robot_name,
        )

        print("Robot Prim:", robot.prim_path)

        return robot