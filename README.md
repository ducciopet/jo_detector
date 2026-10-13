# jo_detector

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-34C759?style=flat-square&logo=ros)](https://docs.ros.org/)
[![Docker](https://img.shields.io/badge/Docker-Supported-2496ED?style=flat-square&logo=docker)](https://www.docker.com/)

Real-time 3D dynamic obstacle detection and tracking for the Jo robot platform. Fuses depth camera, LiDAR, and YOLO 2D detections to produce tracked, classified 3D bounding boxes suitable for downstream navigation and collision avoidance.

Originally derived from Zhefan Xu's [LV-DOT](https://github.com/Zhefan-Xu/LV-DOT) (`onboard_detector`, MIT license); `dbscan`, `uvDetector` and `kalmanFilter` keep its original structure. This package was renamed from `onboard_detector` to `jo_detector` to avoid confusion with the upstream package.

---

## Architecture overview

```
  ┌─────────────────────────────────────────────────────────────────┐
  │                      SENSOR INPUTS                              │
  └────────────┬───────────────────┬──────────────────┬────────────┘
               │ RGB + Depth        │ LiDAR scan        │ Odometry
               ▼                   │                   │
  ┌────────────────────────┐       │                   │
  │  yolov11_detector_node │       │                   │
  │  (Python, GPU/CPU)     │       │                   │
  │  YOLOv11 inference     │       │                   │
  └───────────┬────────────┘       │                   │
              │ /yolo_detector/     │                   │
              │ detected_bboxes     │                   │
              ▼                   ▼                   ▼
  ┌──────────────────────────────────────────────────────────────┐
  │                       detector_node                          │
  │                                                              │
  │  depth ──► uvDetect()     ─────┐                            │
  │  depth ──► dbscanDetect() ─────┼──► filterLVBBoxes()        │
  │  lidar ──► lidarDetect()  ─────┘    (fuse + YOLO assoc)     │
  │                                          │                   │
  │  ◄── ground/walls (wall_detector) ───────┤                   │
  │  ◄── odometry / TF ──────────────────────┤                   │
  │                                          ▼                   │
  │                            kalmanFilterAndUpdateHist()        │
  │                            (predict · associate · update)     │
  │                                          │                   │
  │                                          ▼                   │
  │                              classificationCB()              │
  │                              (dynamic / potentially dynamic)  │
  │                                          │                   │
  │                                          ▼                   │
  │                                       visCB()               │
  │                             publish bboxes & obstacles       │
  └──────────────────────────────────────────────────────────────┘
          ▲                              ▲
          │ ground height                │ odom → base_link TF
  ┌───────┴──────────┐         ┌─────────┴──────────────┐
  │ wall_detector    │         │ odometry_tf_publisher  │
  │    _node         │         │    _node  (optional)   │
  └───────┬──────────┘         └────────────────────────┘
          │
  ┌───────┴──────────────────────────────────────────────┐
  │              calibration_icp_node                    │
  │  Multi-scene point-to-plane ICP (LiDAR ↔ depth).    │
  │  Outdoor: after icp_timeout_sec, publishes average   │
  │  of valid results (or initial guess if none).        │
  └──────────────────────────────────────────────────────┘
```

See [docs/framework_architecture.md](docs/framework_architecture.md) for a detailed description of every processing stage.

---

## Nodes

### `detector_node`

The main perception node. Runs the full UV + DBSCAN + LiDAR detection, sensor fusion, YOLO association, Kalman filtering, and dynamic classification pipeline.

**Subscribed topics**

| Topic | Type | Description |
|---|---|---|
| `depth_image_topic` | `sensor_msgs/Image` | 16-bit depth image (CV_16UC1) |
| `color_image_topic` | `sensor_msgs/Image` | RGB color image |
| `lidar_pointcloud_topic` | `sensor_msgs/PointCloud2` | 3D LiDAR scan |
| `odom_topic` / `pose_topic` | `nav_msgs/Odometry` or `geometry_msgs/PoseStamped` | Robot localization |
| `/yolo_detector/detected_bounding_boxes` | `vision_msgs/Detection2DArray` | 2D YOLO detections |
| `/jo_detector/ground_height` | `std_msgs/Float64MultiArray` | Ground and roof height from `wall_detector_node` |

**Published topics**

| Topic | Type | Description |
|---|---|---|
| `/jo_detector/filtered_bboxes` | `visualization_msgs/MarkerArray` | All tracked 3D boxes |
| `/jo_detector/dynamic_bboxes` | `visualization_msgs/MarkerArray` | Confirmed dynamic obstacles (red) |
| `/jo_detector/potentially_dynamic_bboxes` | `visualization_msgs/MarkerArray` | YOLO-identified but near-stationary (green) |
| `/jo_detector/dynamic_obstacles` | `jo_msgs/ObstacleArray` | Typed message array for navigation |
| `/jo_detector/predicted_bboxes` | `visualization_msgs/MarkerArray` | Kalman-predicted future positions |
| `/yolo_points` | `sensor_msgs/PointCloud2` | Depth points inside YOLO detections (debug) |
| `/filtered_before_yolo_bboxes` | `visualization_msgs/MarkerArray` | Fused 3D boxes before YOLO association (debug) |

---

### `wall_detector_node`

Runs RANSAC on the LiDAR scan to detect vertical planes (walls) and estimates the ground height from the depth image via a separate RANSAC pass. Results are published to `detector_node` to keep its height-based filters calibrated dynamically.

**Subscribed topics:** LiDAR point cloud, depth image

**Published topics:**
- `/jo_detector/ground_height` — `std_msgs/Float64MultiArray` `[ground_height, roof_height]`
- `/jo_detector/wall_bboxes` — `visualization_msgs/MarkerArray` oriented wall bounding boxes
- Filtered LiDAR cloud and debug depth cloud (for RViz)

**Key behaviours:**
- Detection is gated on the `camera_refined` TF being available (published by `calibration_icp_node`).
- Wall OBBs are tracked across frames in a `WallBBoxRegistry` that applies EMA merging and frame-based expiry.
- Ground height is smoothed with an EMA filter (`ground_ema_alpha`) to reduce sensor noise.

---

### `calibration_icp_node`

One-shot extrinsic calibration between the LiDAR and the depth camera. Collects up to `icp_max_runs` synchronized (LiDAR, depth) scene pairs, runs point-to-plane ICP on each, filters results by fitness score, and publishes the **averaged** refined transform as a static TF.

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `icp_max_runs` | 3 | Number of independent scenes to collect before averaging |
| `icp_fitness_threshold` | 0.5 | Min ICP fitness score to accept a result (0–1) |
| `icp_max_correspondence_distance` | 0.2 m | Max point-to-point distance for ICP correspondences |
| `icp_use_point_to_plane` | true | Point-to-plane ICP (requires normal estimation) |
| `icp_voxel_size` | 0.05 m | Voxel down-sampling before ICP |
| `icp_normal_radius` | 0.15 m | Search radius for normal estimation |
| `icp_normal_max_nn` | 30 | Max neighbours for normal estimation |
| `min_overlap_points` | 100 | Min LiDAR points in FOV; if fewer, full cloud is used as target |
| `icp_timeout_sec` | 10.0 s | Outdoor fallback timeout (see below) |
| `is_indoor` | false | Switches ICP timeout behaviour |

**Outdoor timeout behaviour (`is_indoor: false`):**
After `icp_timeout_sec` seconds, the node falls back gracefully:
- If ≥ 1 valid ICP result was collected → **publishes the average** of those results as `camera_refined`.
- If 0 valid results → publishes the **initial-guess TF** so detection can start immediately.

**Published TF:** `velodyne → camera_refined`

---

### `yolov11_detector_node.py`

Runs YOLOv11 inference on the color image at ~30 Hz and publishes 2D bounding boxes with class labels. Target classes are configurable (default: person, car, bus, truck, motorbike, bicycle, and common animals).

**Parameters:** `weights_path`, `inference_size` (640 px), `timer_period_sec`, `target_classes`

---

### `odometry_tf_publisher_node` _(optional)_

Republishes `/odometry/filtered` as an `odom → base_link` TF transform. Only needed when `jo_navigation`'s EKF node is **not** running alongside the detector (e.g., when playing bags without the full navigation stack).

Launch with `odom_pub:=true`.

---

## Launch

```bash
ros2 launch jo_detector run_detector.launch.py
```

Optional arguments:

| Argument | Default | Description |
|---|---|---|
| `odom_pub` | `false` | Also start the odometry TF publisher |

The launch file starts all five nodes: `calibration_icp_node`, `detector_node`, `wall_detector_node`, `yolov11_detector_node`, and `rviz2`. It also publishes the static initial-guess TF `velodyne → camera_initial_guess` from the hardcoded values at the bottom of the file (update these after each physical camera remount).

---

## Configuration

All parameters are loaded from a single YAML file selected inside `run_detector.launch.py`:

| File | Use case |
|---|---|
| `cfg/detector_param_jo_zotac_outdoor.yaml` | Outdoor — wider range, tuned for cars, relaxed ICP, no depth background filter |
| `cfg/detector_param_jo_zotac_indoor.yaml` | Indoor — tighter range, 10th-percentile depth background filter active |

### Key parameter groups

| Group | Relevant params |
|---|---|
| Detection ranges | `local_lidar_range`, `local_sensor_range` |
| LiDAR DBSCAN | `lidar_DBSCAN_epsilon`, `lidar_DBSCAN_min_points` |
| Visual DBSCAN | `dbscan_search_range_epsilon`, `dbscan_min_points_cluster` |
| Kalman filter | `kalman_filter_v2_param` `[eP, eQPos, eQVel, eQAcc, eRPos]` |
| Association | `max_match_range`, `max_match_speed`, scoring weights |
| YOLO matching | `yolo_point_fraction_threshold`, `yolo_2d_iou_threshold` |
| Classification | `dynamic_velocity_threshold`, `dynamic_voting_threshold` |
| ICP calibration | `icp_fitness_threshold`, `icp_timeout_sec`, `icp_max_correspondence_distance` |
| Environment | `is_indoor` (depth filter + ICP timeout behaviour) |

### Outdoor ICP — recommended relaxed values

Outdoor scenes have sparser point clouds and fewer planar surfaces than indoor environments. The outdoor config ships with the following relaxed values compared to the indoor defaults:

| Parameter | Indoor default | Outdoor value |
|---|---|---|
| `icp_fitness_threshold` | 0.5 | 0.15 |
| `icp_max_correspondence_distance` | 0.2 m | 0.35 m |
| `icp_voxel_size` | 0.05 m | 0.08 m |
| `icp_normal_radius` | 0.15 m | 0.25 m |
| `icp_normal_max_nn` | 30 | 50 |
| `min_overlap_points` | 100 | 50 |
| `icp_timeout_sec` | — (no timeout) | 15.0 s |

---

## Custom messages

### `jo_msgs/Obstacle.msg`
Per-obstacle message: 3D position, velocity, bounding box dimensions, YOLO flag, dynamic flag.

### `jo_msgs/ObstacleArray.msg`
Stamped array of `Obstacle`.

### `GetDynamicObstacles.srv`
Service to query the current dynamic obstacle list on demand.

---

## Building

```bash
# Inside the Docker container
source /opt/ros/jazzy/setup.bash
source /home/ros/local_install/setup.bash

# First build or after a directory/symlink conflict:
rm -rf /home/ros/build/jo_detector /home/ros/local_install/jo_detector

colcon build \
    --packages-select jo_detector \
    --symlink-install \
    --install-base /home/ros/local_install

source /home/ros/local_install/setup.bash
```

Python files and launch scripts are symlinked at build time, so changes to `.py` files and YAML configs take effect immediately without rebuilding. C++ changes (`src/`, `include/`, `scripts/wall_detector/`) require a rebuild.
