# vision_pick

ROS 2 HSV colour segmentation and RGB-D back-projection for coloured wooden rods in Isaac Sim, picked up with a UR7e arm through MoveIt (the `pick_place_node` calls `/move_action`; whichever planning pipeline `ur_moveit_config`'s launch file configures — e.g. cuMotion — handles the actual planning).

## Build

From the ROS 2 workspace that contains this repository:

```bash
colcon build --packages-select vision_pick
source install/setup.bash
```

## Run

```bash
ros2 run vision_pick segmentation_node --ros-args -p target_color:=red
```

Available colours are `red`, `orange`, `yellow`, `green`, `blue`, and `purple`. The camera topic names can be overridden with `rgb_topic`, `depth_topic`, and `camera_info_topic` parameters.

The node synchronizes RGB and depth with `ApproximateTimeSynchronizer`, then publishes:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/detected_pixel` | `geometry_msgs/Point` | `x=u`, `y=v` for the target colour's centroid in pixels; `z=0`. |
| `/vision_pick/mask` | `sensor_msgs/Image` | HSV binary segmentation mask (`mono8`) for the target colour only. |
| `/detected_object_pose` | `geometry_msgs/PoseStamped` | Target rod-centre position in the camera optical frame, in metres. Orientation is identity because colour segmentation does not estimate rotation. |
| `/detected_objects` | `geometry_msgs/PoseArray` | Every recognised colour's position this frame (target included), in the camera optical frame. |
| `/detected_object_sizes` | `std_msgs/Float64MultiArray` | Flat `[w0,h0,d0, w1,h1,d1, ...]`, same order as `/detected_objects`' poses. Each blob's own pixel bounding box converted to metres via its measured depth and the camera intrinsics (`size = size_px / focal_px * depth`); the vertical (table-normal) extent isn't observable from a top-down camera, so it's approximated as the narrower of the two measured sides. |

Alongside the colour blobs, every cycle also runs `surface_detector.detect_surfaces` on the depth image: it bins valid depth readings and scans them nearest-first, taking each one big enough to plausibly be a real surface (a table, a box sitting on it, a shelf, ...) rather than just whichever depth has the most pixels overall — otherwise a farther-away but larger plane (e.g. the floor visible past the table's edge) can beat the table itself, and a nearer smaller one (a box on the table) would get lost inside the table's own plane instead of being its own obstacle. Each surface found (nearest first, up to `surface_max_count`, default `4`) gets appended to `/detected_objects`/`/detected_object_sizes` as one more entry — there's no colour, position, or size for "the table" or "the box" hard-coded anywhere; each is found and sized the same way any other flat surface would be. Its `surface_depth_tolerance` (default `0.02` m), `surface_min_area_fraction` (default `0.15`), and `surface_thickness` (default `0.05` m, since a flat surface's own thickness isn't observable from one side) parameters control that detector.

`pick_place_node` combines `/detected_objects` and `/detected_object_sizes` to register every detected object except the current target — rods, the detected surface, whatever else shows up — as a MoveIt collision obstacle, sized and positioned entirely from what the camera currently sees; nothing is hand-entered. Its `rod_exclude_radius` parameter (default `0.10` m) is the only knob: the distance within which a detected blob is assumed to be the current target rather than an obstacle.

This sync runs on a 0.5s timer (`_periodic_collision_sync`), independent of whether a pick is actively in progress — using whichever target was last detected (or excluding nothing, if none has ever been seen) — so the planning scene reflects the table/rods even before a pick starts, not only during the brief window a pick sequence is running.

Since the camera is mounted on the wrist, it only sees the actual workspace while the arm is still near its start pose — once the arm moves off to pick, it's pointed wherever the wrist happens to be facing, not necessarily the table. So this periodic sync only runs for `collision_scan_seconds` (default `3.0` s) after the node starts, then freezes whatever was last registered rather than continuing to replace it with whatever the camera currently sees. (The scene still gets refreshed once per actual pick attempt, right before planning, via `_sync_rod_collision_objects` — that one's driven by the just-detected target, not the timer.)

`pick_place_node`'s `planning_pipeline_id` parameter is empty by default, so `/move_action` uses whichever pipeline `ur_moveit_config`'s launch sets as default; set it only if you need to force a specific named pipeline.

For visualization, add `/vision_pick/mask` as an Image display in RViz. Verify the data flow with:

```bash
ros2 topic echo /detected_pixel
ros2 topic echo /detected_object_pose
```

`/camera/camera_info` must contain valid intrinsics. A `16UC1` depth image is interpreted as millimetres; other depth encodings are interpreted as metres (the usual `32FC1` convention).
