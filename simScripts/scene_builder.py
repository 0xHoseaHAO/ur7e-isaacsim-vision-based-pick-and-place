"""
scene_builder.py

Scene builder for Isaac Sim 6.0.1

Responsible for creating the simulation environment.
"""

from __future__ import annotations

from pxr import UsdLux

from isaacsim.core.api import World


class SceneBuilder:
    """
    Build the simulation scene.

    Responsibilities
    ----------------
    - Ground plane
    - Lighting
    - Environment assets (future)
    """

    def __init__(self, world: World):

        self.world = world
        self.stage = world.stage

    def build(self):

        self._create_ground()
        self._create_lighting()

    def _create_ground(self):

        self.world.scene.add_default_ground_plane()

        print("✓ Ground plane created")

    def _create_lighting(self):

        light = UsdLux.DomeLight.Define(
            self.stage,
            "/World/DomeLight"
        )

        light.CreateIntensityAttr(2500)

        print("✓ Dome light created")

    def load_environment_usd(
        self,
        usd_path: str,
        prim_path="/World/Environment"
    ):
        """
        Placeholder.

        Next version:
            Load warehouse/table USD.
        """

        print(f"Environment USD: {usd_path}")