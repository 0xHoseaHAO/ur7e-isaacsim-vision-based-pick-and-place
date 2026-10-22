# Vision-Based Pick and Place

Isaac Sim + ROS 2 pick-and-place: a **UR7e** arm, wrist RGB-D camera, and
perception that feeds **MoveIt 2** (cuMotion-capable) for grasping coloured
rods / COCO objects in simulation.

| Piece | Role |
|---|---|
| [`simScripts/`](simScripts/) | Isaac Sim bringup (scene, UR7e, ROS 2 bridge, camera, gripper) |
| [`vision_pick/`](vision_pick/) | ROS 2 package: HSV / YOLO perception + `pick_place_node` |
| [`assets/`](assets/) | USD props + UR7e URDF used by the sim |
| [`ur_isaac_sim.launch.py`](ur_isaac_sim.launch.py) | MoveIt / UR launch (cuMotion via MoveIt configs) |

Package-level detail (topics, parameters, collision sync):  
→ [`vision_pick/README.md`](vision_pick/README.md)

## Requirements

- NVIDIA Isaac Sim **6.x** (bringup comments target 6.0.1+)
- ROS 2 (Humble or later) with MoveIt 2 / `ur_moveit_config`
- Python deps for perception: OpenCV, NumPy; for YOLO: `pip install ultralytics`
- GPU with a current NVIDIA driver

## Layout tip

Treat this repository as a ROS 2 package source folder (or symlink
`vision_pick/` into your colcon workspace `src/`). Keep Isaac Sim scripts under
`simScripts/` and run them with Isaac’s `python.sh` / Script Editor as you
normally would.

## Quick start

### 1. Build the ROS 2 package

```bash
# from your ROS 2 workspace that contains vision_pick/
colcon build --packages-select vision_pick
source install/setup.bash
```

### 2. Start Isaac Sim scene

```bash
# From this repo root, with Isaac Sim’s Python environment:
# (exact launcher depends on your Isaac install)
./python.sh simScripts/bringup.py
```

`bringup.py` loads the table/props, UR7e, enables the ROS 2 bridge, and
publishes camera + joint interfaces.

### 3. Start MoveIt + perception + pick

```bash
ros2 launch ur_isaac_sim.launch.py   # or your UR MoveIt launch
ros2 run vision_pick segmentation_node --ros-args -p target_color:=red
# optional YOLO instead of HSV (downloads weights on first run):
# ros2 run vision_pick yolo_detection_node --ros-args -p model_path:=yolo26m-seg.pt
ros2 run vision_pick pick_place_node
```

HSV colours: `red`, `orange`, `yellow`, `green`, `blue`, `purple`.

## YOLO weights

Checkpoint `.pt` files are **not** stored in git. Ultralytics will download
stock checkpoints (e.g. `yolo26m-seg.pt`, `yolov8n-seg.pt`) on first use when
`model_path` points at a known model name. Place custom weights next to the
node or pass an absolute `model_path` if needed.

## License

- `vision_pick`: Apache-2.0 (see `vision_pick/package.xml`)
- `ur_isaac_sim.launch.py`: NVIDIA Apache-2.0 + upstream MoveIt BSD notices in-file
- Assets / third-party USD may carry their own licenses — check each asset tree before redistribution
