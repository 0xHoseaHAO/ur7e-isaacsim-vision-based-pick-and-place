"""
robot_loader.py

Generic URDF loader for Isaac Sim 6.0.1 (Optimized for Relative Paths)
"""

from pathlib import Path

import omni.usd
from pxr import Gf, UsdGeom

from isaacsim.asset.importer.urdf import (
    URDFImporter,
    URDFImporterConfig,
)

from isaacsim.core.utils.stage import add_reference_to_stage


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
        """
        Import a URDF and spawn it into the current stage.
        """

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
        config.force_override = True  # 覆盖旧缓存，防止脏数据影响

        #
        # Convert URDF → USD
        #
        importer = URDFImporter(config)

        print("=" * 60)
        print("Importing URDF with local relative paths...")

        usd_path = importer.import_urdf()

        print("Generated USD:")
        print(usd_path)

        #
        # Spawn USD
        #
        prim_path = f"/World/{robot_name}"

        # 避免旧节点冲突，在挂载前如果存在旧节点则先删除
        stage = omni.usd.get_context().get_stage()
        if stage.GetPrimAtPath(prim_path):
            omni.kit.commands.execute('DeletePrim', path=prim_path)

        add_reference_to_stage(
            usd_path=usd_path,
            prim_path=prim_path,
        )

        #
        # Move robot
        #
        prim = stage.GetPrimAtPath(prim_path)

        if not prim.IsValid():
            raise RuntimeError(f"Cannot find prim {prim_path}")

        #
        # Translate only
        #
        xform = UsdGeom.XformCommonAPI(prim)

        xform.SetTranslate(
            Gf.Vec3d(
                position[0],
                position[1],
                position[2],
            )
        )

        print("=" * 60)
        print(f"Robot loaded successfully: {prim_path}")

        print("Children:")

        for child in prim.GetChildren():
            print("  -", child.GetName())

        print("=" * 60)

        return prim_path