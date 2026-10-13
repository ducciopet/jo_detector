# jo_detector — Framework Architecture

This document describes the internal logic of every node and processing module in `jo_detector`, from raw sensor data to classified and tracked 3D bounding boxes published to the navigation stack.

---

## Table of contents

1. [System-level data flow](#1-system-level-data-flow)
2. [calibration_icp_node](#2-calibration_icp_node)
3. [wall_detector_node](#3-wall_detector_node)
4. [detector_node — overview](#4-detector_node--overview)
5. [UV detection (uvDetect)](#5-uv-detection-uvdetect)
6. [Visual DBSCAN (dbscanDetect)](#6-visual-dbscan-dbscandetect)
7. [LiDAR detection (lidarDetect)](#7-lidar-detection-lidardetect)
8. [Fusion and YOLO association (filterLVBBoxes)](#8-fusion-and-yolo-association-filterlvbboxes)
9. [Kalman filter and track management](#9-kalman-filter-and-track-management)
10. [Dynamic classification stage](#10-dynamic-classification-stage)
11. [Outside-FOV association and classification](#11-outside-fov-association-and-classification)
12. [yolov11_detector_node](#12-yolov11_detector_node)
13. [odometry_tf_publisher_node](#13-odometry_tf_publisher_node)
14. [ROS outputs and status semantics](#14-ros-outputs-and-status-semantics)
15. [Indoor vs outdoor mode](#15-indoor-vs-outdoor-mode)
16. [Tuning reference](#16-tuning-reference)
17. [Debugging guide](#17-debugging-guide)

---

## 1. System-level data flow

The pipeline is **timer-driven**, not callback-driven, but the current architecture is no longer split across five independent processing timers. The synchronized sensor callbacks update shared state buffers; one main wall timer (`mainTimer_`, period `time_step`, default 0.1 s = 10 Hz) consumes the latest buffers and runs the perception pipeline in a deterministic order. Inside that main callback, the expensive detector front-ends run in parallel worker threads.

```
SENSOR CALLBACKS (update shared state, fire at sensor rate)
─────────────────────────────────────────────────────────────────────
ApproximateTime(depth, odom)  ──► depthOdomCB()
                                    store depthImage_, camera pose

ApproximateTime(LiDAR, odom)  ──► lidarOdomCB()
                                    range-filter, Gaussian downsample,
                                    transform to world frame, wall-filter
                                    → store lidarCloud_, lidar pose

color image  ──────────────────► colorImgCB()     store detectedColorImage_
YOLO 2D      ──────────────────► yoloDetectionCB() store yoloDetectionResults_
ground height ─────────────────► groundHeightCB()  store groundHeight_, roofHeight_
wall markers ──────────────────► wallMarkersCB()   store wallBBoxes_


MAIN TIMER PIPELINE (mainTimer_, every time_step = 0.1 s)
─────────────────────────────────────────────────────────────────────
mainCB()
  ├─ parallel detector front-end
  │    ├─ thread 1: dbscanDetect()   depth → DBSCAN clusters
  │    ├─ thread 2: uvDetect()       depth → UV occupancy boxes
  │    └─ thread 3: lidarDetect()    lidarCloud_ → DBSCAN clusters
  │
  ├─ join all detector threads
  ├─ filterLVBBoxes()                fuse UV+DBSCAN+LiDAR + YOLO assoc
  ├─ tracking pre-filter             remove boxes whose bottom is too high
  ├─ boxAssociation()
  ├─ kalmanFilterAndUpdateHist()      predict · associate · update / create / drop
  └─ classification stage             PATH 1/2/3 dynamic classification

VIS TIMER (visTimer_, every time_step = 0.1 s)
─────────────────────────────────────────────────────────────────────
visCB()
  └─ publish MarkerArrays + ObstacleArray
```

The decoupling between sensor callbacks and `mainCB()` means that the detection pipeline operates on the **latest available** buffered data from each sensor, without needing to triple-synchronize depth, LiDAR, and odometry in a single message filter. Depth and LiDAR are each synchronized with odometry/pose independently, so the detector can keep a pose buffer for each sensor stream.

Implementation detail: `depthOdomCB()` / `depthPoseCB()` compute the depth/color camera pose before storing depth-derived state. `lidarOdomCB()` / `lidarPoseCB()` currently apply the LiDAR transform using the already-buffered `positionLidar_` / `orientationLidar_`, then refresh those fields from the current odometry/pose at the end of the callback. At normal rates this is usually a one-callback pose lag, but it is worth remembering when debugging fast motion or timestamp alignment.

This is a **single-orchestrator, multi-worker** design:

- `mainCB()` owns the stage ordering. Fusion never starts until DBSCAN, UV, and LiDAR detection threads have all returned.
- `dbscanDetect()`, `uvDetect()`, and `lidarDetect()` are independent enough to run concurrently because they write separate stage buffers (`dbBBoxes_`, `uvBBoxes_`, `lidarBBoxes_`).
- Tracking and classification remain sequential after fusion, because they mutate shared track history, Kalman filters, and classification flags.
- `visCB()` runs on its own timer so visualization/publication is decoupled from the heavier detection/tracking path.
- `mainCB()` returns early until the LiDAR-to-depth-camera calibration is available (`lidarToDepthCamOk_`). This prevents the detector from fusing uncalibrated camera/LiDAR geometry.

### Coordinate-frame convention

`detector_node` publishes object-level outputs in the odometry/world frame used by the detector, currently `odom` for point clouds and marker arrays. Sensor-local data is converted before fusion:

- Depth-derived boxes start in the camera frame, then are transformed by the pose buffered in `depthOdomCB()` / `depthPoseCB()`.
- LiDAR points are transformed in `lidarOdomCB()` / `lidarPoseCB()` from LiDAR frame to world frame before DBSCAN. As noted above, this uses the currently buffered LiDAR pose and then refreshes that pose from the synchronized odometry/pose message.
- YOLO depth points are unprojected in the depth camera frame and then transformed to world frame before testing against 3D boxes.

This means the tracker, classifier, wall filtering, and YOLO association all operate on a single world-frame geometry. The only image-frame operation that remains image-native is the 2D YOLO rectangle and the 2D IoU fallback.

### Package-level responsibilities

`jo_detector` deliberately separates perception into several nodes:

- `calibration_icp_node` provides a refined camera/LiDAR extrinsic.
- `wall_detector_node` provides ground height, roof height, and persistent wall OBBs.
- `detector_node` performs object detection, fusion, tracking, dynamic classification, and ROS publication.
- `yolov11_detector_node` produces class-filtered 2D detections.
- `odometry_tf_publisher_node` is a utility for sessions where the EKF TF is not already being broadcast.

The navigation stack consumes the high-level `jo_msgs/ObstacleArray` published by `detector_node`, while RViz mostly consumes MarkerArrays and debug point clouds.

---

## 2. calibration_icp_node

### Purpose

The depth camera and LiDAR have mechanically independent coordinate frames. `calibration_icp_node` computes the rigid transform **velodyne → camera_refined** by aligning the depth point cloud (expressed in the LiDAR frame via an initial guess) with the LiDAR scan itself. This transform is published as a static TF and used by all other nodes for the lifetime of the session.

### Initialization

At startup the node reads the initial-guess TF `velodyne → camera_initial_guess` from the TF tree. This transform is published by a `static_transform_publisher` in the launch file and should encode a rough manually-measured extrinsic (e.g., from a previous ICP run or a physical measurement). All ICP runs use this as the starting point for iterative refinement.

### Per-scene ICP pipeline

Each time a synchronized (LiDAR scan, depth image) pair arrives:

1. **Depth → point cloud.** The depth image is projected to 3D using the pinhole camera model and depth intrinsics. Pixels with depth outside `[depth_min_value, depth_max_value]` are discarded. Every `depth_skip_pixel`-th pixel is used to reduce computation.

2. **LiDAR overlap extraction.** The full LiDAR cloud is filtered to keep only points that project into the depth camera's field of view (with a margin of `overlap_fov_margin_px` pixels). If fewer than `min_overlap_points` survive, the full LiDAR cloud is used as the ICP target to prevent degenerate registration.

3. **Voxel down-sampling.** Both the depth cloud (source) and the LiDAR overlap (target) are voxel-filtered at `icp_voxel_size` to normalize point density.

4. **Normal estimation (point-to-plane mode).** When `icp_use_point_to_plane: true`, surface normals are estimated on the target cloud using a KD-tree search within radius `icp_normal_radius` and at most `icp_normal_max_nn` neighbours. Larger values are needed for outdoor scenes where the point density is lower.

5. **ICP registration.** Open3D's `RegistrationICP` runs for up to `icp_max_iter` iterations with a maximum correspondence distance of `icp_max_correspondence_distance`. The initial transformation is the current estimate of `T_lidar_camera_init_` (starts from the initial-guess TF, updated after each accepted scene).

6. **Fitness gate.** The ICP fitness score (fraction of source points with a correspondence below the distance threshold) is checked against `icp_fitness_threshold`. Scenes that fail are discarded. Accepted scenes contribute their transform and RPY to the accumulator.

### Multi-scene averaging

After `icp_max_runs` scenes have been processed, the node computes the final extrinsic:

- **Translation:** arithmetic mean of accepted translation vectors.
- **Rotation:** mean-of-angles using `atan2(mean_sin, mean_cos)` per roll/pitch/yaw axis, which correctly handles angle wraparound.

The averaged transform is then published as the static TF `velodyne → camera_refined` and logged in full.

If no scene ever passes the fitness threshold (all discarded), the accumulator is reset and the node retries from the next incoming frame batch.

### Outdoor timeout and fallback

When `is_indoor: false`, a wall timer fires after `icp_timeout_sec` seconds:

- **If ≥ 1 valid ICP result exists:** `publishOnTimeout()` calls `finalizeCalibration()`, which publishes the partial average as `camera_refined`. Detection starts with a coarser but real ICP estimate.
- **If 0 valid results exist:** the initial-guess TF is published directly as `camera_refined`. Detection starts immediately with no refinement.

In both cases `processed_ = true` is set, which silences all further callbacks. A nested retry timer at 1-second intervals handles the case where the TF lookup fails transiently at timeout time (e.g., the clock is not yet running with `use_sim_time`).

The indoor path has no timeout: the node waits indefinitely for a full set of `icp_max_runs` valid scenes.

### Why ICP is harder outdoors

Outdoor environments typically have:
- **Fewer planar surfaces** for point-to-plane ICP to anchor onto (no walls/floor/ceiling forming a complete geometric cage).
- **Sparser depth returns** from sunlight interference or long-range objects.
- **Less overlap** between the LiDAR and depth camera FOVs when the scene is open.

The outdoor config compensates by relaxing `icp_fitness_threshold` (0.15 vs 0.5), widening `icp_max_correspondence_distance` (0.35 m vs 0.2 m), using larger voxels (0.08 m) and a larger normal estimation radius (0.25 m).

---

## 3. wall_detector_node

### Purpose

`wall_detector_node` has two independent responsibilities that run in parallel:

1. **Ground height estimation** from the depth camera — feeds `detector_node` so its height-based filters always track the true ground level even on sloped or uneven terrain.
2. **Wall detection** from the LiDAR — maintains a registry of vertical planar surfaces so `detector_node` can filter out static wall detections from its obstacle output.

### Calibration gate

Both functions are gated on the `camera_refined` TF being present in the TF tree (published by `calibration_icp_node`). Until that TF appears, the node silently drops all incoming messages. This prevents using a stale or zero extrinsic during the calibration phase.

### Ground height estimation (depth RANSAC)

**Why from depth and not LiDAR?** The depth camera gives a dense, textured image of the floor surface in front of the robot and is less affected by specular reflections from flat surfaces. LiDAR often under-samples the ground at close range due to the scan angle.

**Algorithm:**

1. **ROI selection.** Only the bottom `1/ground_estim_bottom_fraction` rows of the depth image are used. These rows correspond to the ground plane in front of the robot at typical mounting heights.
2. **3D projection.** Each valid depth pixel in the ROI is unprojected to a 3D point in the depth camera frame using the pinhole model.
3. **Transform to world frame.** Points are transformed using the current `camera_refined` pose (from TF).
4. **RANSAC plane fit.** Three random points define a candidate plane. Inliers are all points within a distance threshold of the plane. The best plane (most inliers) after `ransac_max_iterations` is accepted if it has at least `ground_estim_min_inliers` points.
5. **Ground height update.** The median Z of inlier points plus `ground_conservative_offset` is used as the new ground height estimate. This conservative offset ensures the ground is never estimated too low (which would cause ground-level points considered as obstacles from clustering algorithms).
6. **EMA smoothing.** The published ground height is smoothed with an EMA filter: `g_new = α * g_raw + (1−α) * g_prev`. A small `ground_ema_alpha` (e.g., 0.05) prevents sudden jumps from individual noisy depth frames.

The roof height is always `ground_height + ground_roof_offset`.

### Wall detection (LiDAR RANSAC)

**Algorithm:**

1. **Voxel filtering.** The incoming LiDAR cloud is voxel-filtered at `voxel_resolution` to normalize point density and reduce computation.
2. **Iterative plane extraction.** RANSAC is run repeatedly on the remaining cloud until `max_planes` have been found or no more planes meet the minimum inlier count:
   a. Three random points define a candidate plane via `fitPlane()`.
   b. All points within `ransac_inlier_threshold` of that plane are counted as inliers.
   c. After `ransac_max_iterations` samples, the best plane is accepted if it has ≥ `ransac_min_inliers` inliers.
   d. Inlier points are removed from the cloud before the next iteration.
3. **Wall classification.** A plane is classified as a wall if its normal vector is sufficiently horizontal, i.e., the angle between the normal and the XY plane is ≤ `wall_vertical_angle_deg` degrees. Ground and ceiling planes (near-horizontal normals) are discarded.
4. **Contiguous cluster extraction.** The inlier points of each wall plane are further clustered spatially to extract the largest contiguous segment. This removes stray isolated inlier points that are geometrically on the plane but physically disconnected.
5. **OBB fitting.** An oriented bounding box is fitted to the contiguous inlier cluster using PCA on the inlier point set: the eigenvectors of the covariance matrix define the OBB axes, and the extents are the projections along those axes. The OBB is discarded if its aspect ratio (long/short axis) exceeds `wall_bbox_max_aspect_ratio` (which would indicate a degenerate plane fit, not a real wall).

### WallBBoxRegistry — persistent wall tracking

Individual LiDAR scans are noisy and may miss walls due to occlusion or scan angle. The `WallBBoxRegistry` maintains a persistent set of wall OBBs across frames:

- **Matching.** Each newly detected wall OBB is matched to the closest existing registry entry based on: (a) IoV (intersection-over-volume of the smaller box) ≥ `overlap_threshold`, (b) surface normal dot product ≥ `min_normal_dot`, and (c) center distance ≤ `max_center_distance`. All three conditions must hold.
- **Merging.** Matched entries are updated with an EMA: `existing = (1−w) * existing + w * incoming` where `w = merge_weight`. This prevents abrupt jumps while still allowing the registry to adapt to small changes.
- **New entries.** Unmatched detections are added to the registry as new entries.
- **Pose compensation.** Between frames, all existing registry OBBs are transformed by the odometry delta pose (current pose − previous pose) to keep them aligned with the world frame even as the robot moves.
- **Expiry.** Entries not matched for `max_missed_frames` consecutive frames are removed. This allows walls to disappear when the robot moves past them or a doorway opens.

---

## 4. detector_node — overview

`detector_node` instantiates `dynamicDetector` and wires its inputs to ROS subscriptions and timers. Sensor data arrives through two independent `ApproximateTime` synchronizers and is buffered in shared members. A single main wall timer at `time_step` then runs the ordered perception pipeline, while spawning three short-lived worker threads for the detector front-end:

```
Sensor callbacks (update shared state only):
  depthOdomCB()        ← ApproximateTime(depth, odom)   → depthImage_, camera pose
  lidarOdomCB()        ← ApproximateTime(LiDAR, odom)   → lidarCloud_, lidar pose
  colorImgCB()         ← color image topic              → detectedColorImage_
  yoloDetectionCB()    ← YOLO detections topic          → yoloDetectionResults_
  groundHeightCB()     ← wall_detector ground topic     → groundHeight_, roofHeight_
  wallMarkersCB()      ← wall_detector marker topic     → wallBBoxes_

Timer callbacks:
  mainCB()             → parallel(dbscanDetect, uvDetect, lidarDetect)
                       → filterLVBBoxes()
                       → boxAssociation()
                       → kalmanFilterAndUpdateHist()
                       → PATH 1/2/3 classification

  visCB()              → publish MarkerArrays + ObstacleArray
```

The odom/pose message is paired independently with each sensor so the detector can maintain separate camera and LiDAR pose buffers, without needing to triple-synchronize depth, LiDAR, and odometry in a single message filter. In the current LiDAR callback implementation, the cloud transform uses the previously buffered LiDAR pose and then refreshes that pose from the synchronized message.

### Multithreaded execution model

The ROS executor itself is still launched with `rclcpp::spin(nh)` in `detector_node.cpp`. The detector's parallelism is explicit inside `dynamicDetector::mainCB()`:

```cpp
std::thread t_db([this]{ this->dbscanDetect(); });
std::thread t_uv([this]{ this->uvDetect(); });
std::thread t_li([this]{ this->lidarDetect(); });
t_db.join();
t_uv.join();
t_li.join();
```

This means there is no ROS callback-group based scheduling for the detector stages. Instead, `mainCB()` starts the three detector workers, waits for all of them, and only then continues with fusion, tracking, and classification. The result is parallel front-end computation with a deterministic back-end order.

Wall OBB updates are the one explicitly protected shared structure: `wallBBoxes_` is guarded by `wallBBoxesMutex_` when received from `wall_detector_node` and when copied/used during point filtering. Most other sensor buffers are updated by ROS callbacks and consumed by the next `mainCB()` cycle as latest-value state.

There is also a small service interface, `jo_detector/get_dynamic_obstacles`, which returns nearby entries from `dynamicBBoxes_` sorted by distance from a requested point. It is a convenience/legacy query path over the dedicated dynamic list; the richer and more stable downstream interface is still the `jo_msgs/ObstacleArray` topic.

### Main internal buffers

The detector keeps separate buffers for each stage so that debugging each stage in RViz is possible:

| Buffer | Meaning | Producer | Consumer |
|---|---|---|---|
| `depthImage_` | Latest raw depth image | `depthOdomCB()` | `uvDetect()`, `dbscanDetect()`, YOLO depth association |
| `lidarCloud_` | Pre-filtered world-frame LiDAR cloud | `lidarOdomCB()` | `lidarDetect()` |
| `dbBBoxes_`, `dbPcClusters_` | Visual DBSCAN boxes and point clusters | `dbscanDetect()` | `filterLVBBoxes()` |
| `uvBBoxes_` | U-map boxes | `uvDetect()` | `filterLVBBoxes()` |
| `lidarBBoxes_`, `lidarClusters_` | LiDAR DBSCAN boxes and clusters | `lidarDetect()` | `filterLVBBoxes()` |
| `filteredBBoxesBeforeYolo_` | Fused 3D boxes before YOLO annotation | `filterLVBBoxes()` | RViz/debug |
| `filteredBBoxes_`, `filteredPcClusters_` | Final detections entering tracking | `filterLVBBoxes()` | `mainCB()` tracking stage |
| `boxHist_`, `pcHist_` | Per-track history deques | `kalmanFilterAndUpdateHist()` | association, classification, output |
| `trackedBBoxes_` | Confirmed tracks currently publishable | `kalmanFilterAndUpdateHist()` | classification stage, `visCB()` |
| `dynamicBBoxes_` | Tracks classified as moving dynamic obstacles after optional size constraint | classification stage in `mainCB()` | RViz, dynamic point cloud |
| `potentiallyDynamicBBoxes_` | YOLO dynamic-class tracks that are currently stationary | classification stage in `mainCB()` | RViz, `ObstacleArray` |

Two implementation details are worth keeping in mind:

- `is_dynamic` is deliberately recalculated every classification cycle; a moving object that stops should lose the dynamic label.
- `is_yolo_candidate` is track-level evidence and can persist across frames. It is cleared by explicit decay/guard rules, not simply because YOLO did not fire in the current frame.

### Module contracts

The main modules can be read as explicit contracts:

| Module | Input | Output | Frame | Important side effects |
|---|---|---|---|---|
| `depthOdomCB()` | depth image + pose/odom | `depthImage_`, `positionDepth_`, `orientationDepth_` | camera → odom pose buffer | updates latest depth pose only |
| `lidarOdomCB()` | LiDAR cloud + pose/odom | `lidarCloud_`, raw/downsample debug clouds | velodyne → odom/world | range filter, random distance downsample, wall/height filtering, adaptive voxel; transform uses buffered LiDAR pose then refreshes it |
| `dbscanDetect()` | `depthImage_`, depth pose, wall/height limits | `dbBBoxes_`, `dbPcClusters_`, `filteredDepthPoints_` | odom | visual point-cloud clustering |
| `uvDetect()` | `depthImage_` | `uvBBoxes_`, UV debug images | camera box then transformed downstream | U-map detection |
| `lidarDetect()` | `lidarCloud_` | `lidarBBoxes_`, `lidarClusters_` | odom | LiDAR DBSCAN and size filtering |
| `filterLVBBoxes()` | UV, visual DBSCAN, LiDAR boxes, YOLO detections | `filteredBBoxes_`, `filteredPcClusters_`, `filteredBBoxesBeforeYolo_` | odom | fusion, YOLO flags, optional box resize/height correction |
| `boxAssociation()` | `filteredBBoxes_`, track predictions | `bestMatch` | odom | assignment only, no state mutation |
| `kalmanFilterAndUpdateHist()` | `bestMatch`, detections, previous tracks | histories, filters, `trackedBBoxes_` | odom | creates/deletes tracks, sticky YOLO propagation |
| classification stage in `mainCB()` | confirmed output tracks and histories | flags in `boxHist_`, `dynamicBBoxes_`, `potentiallyDynamicBBoxes_` | odom | dynamic decision logic plus optional dynamic-size filtering |
| `visCB()` | all stage outputs | RViz topics + `ObstacleArray` | odom | machine-readable publication |

This table is often the fastest way to debug the pipeline: if a bad box appears in `filteredBBoxesBeforeYolo_`, the problem is fusion; if it appears only after tracking, the problem is association/history; if its status is unexpected, the problem is classification, sticky YOLO evidence, or output suppression.

---

## 5. UV detection (uvDetect)

### Concept

The UV detector builds a **U-disparity map** from the depth image — a 2D histogram where each column corresponds to an image column range and each row corresponds to a depth bin. A cell is "hot" when many pixels in that image-column band have depth falling in that bin. Contiguous hot runs within a row are segmented and merged across rows to form image-space bounding boxes, which are then back-projected to 3D in the camera frame. The approach avoids full 3D point cloud construction; it operates entirely on the raw 16-bit depth image.

### Algorithm

1. **U-map construction (`extract_U_map`).** The depth image is scanned column by column. For each pixel with a valid depth in `[min_dist, max_dist]` mm, the depth is quantised into one of `histSize = depth.rows / row_downsample` bins, and the corresponding U-map cell `(col / col_scale, bin)` is incremented. A `(5, 9)` Gaussian blur (σ = 10) is applied to smooth the histogram.

2. **Bounding-box extraction (`extract_bb`).** Each row of the U-map is scanned left-to-right for runs of cells whose count exceeds `u_min = threshold_point × row_downsample`. Each run becomes a `UVbox` segment. Vertically adjacent segments in consecutive rows that overlap horizontally are merged into the same parent using union-find. After merging, boxes with area < 25 cells are discarded. The result is a list of `bounding_box_U` rectangles in U-map space `(col_start, col_end, bin_start, bin_end)`.

3. **3D bounding-box extraction (`extract_3Dbox`, called from `uvDetect`).** For each U-map bbox:
   - The depth range `[depth_near, depth_far]` is recovered from the bin indices.
   - The image columns `[x_left, x_right]` are recovered from the col indices (× `col_scale`).
   - The vertical extent `[y_up, y_down]` is found by scanning those depth-image columns for the topmost and bottommost pixels whose depth falls in `[depth_near, depth_far]`, using a forward-look of `num_check = 15` consecutive rows to suppress isolated noise.
   - The 3D center in the camera frame is computed via the pinhole back-projection: `X = (col_center − cx) × z / fx`, `Y = (row_center − cy) × z / fy`, `Z = (depth_near + depth_far) / 2`, with depth converted from mm to m.
   - Width and height are derived from the pixel extents scaled by `z / focal_length`.

4. **Bird's-eye visualization (`extract_bird_view`).** Converts U-map bboxes to top-down rectangles for RViz display. This output is **not** used by the 3D detection pipeline.

### Notes

- There is **no ground/roof filtering** inside the UV detector itself; height gating is applied later in `filterLVBBoxes`.
- The 3D boxes come out in the **camera frame**, not the world frame; `filterLVBBoxes` transforms them via the stored camera-to-world pose.
- The UV detector is complementary to DBSCAN: it is faster and works on the raw depth image, but can merge objects at similar depth that are separated horizontally, or split a single object that spans multiple depth bins.

---

## 6. Visual DBSCAN (dbscanDetect)

### Concept

The visual DBSCAN detector projects the depth image to a full 3D point cloud in world space and clusters it with DBSCAN. Unlike the UV detector, which only captures per-column depth histograms, DBSCAN works on individual 3D points and can resolve spatially separated objects at the same depth, distinguish objects with irregular or elongated shapes, and leverage 3D density information for post-processing refinement.

### Algorithm

1. **Depth projection (`projectDepthImage`).** Every valid depth pixel (in `[depth_min_value, depth_max_value]`) is back-projected to the camera frame using the pinhole model, then transformed to the world frame via the stored camera pose (`orientationDepth_`, `positionDepth_` from the last `depthOdomCB`). Points beyond `local_sensor_range` in X or Y are rejected early.

2. **Voxel filtering (`voxelFilter`).** The projected cloud is discretised into voxels of side `voxel_size`. A voxel is kept only when it accumulates at least `voxel_occupied_thresh` raw points — this simultaneously down-samples the cloud and rejects isolated noisy readings without discarding coherent but low-density returns.

3. **Ground/roof and wall filtering (`filterPoints`).** After voxel filtering, points outside `[groundHeight_, roofHeight_]` in Z are discarded, and points that fall inside any known wall OBB (`isInsideAnyWall`) are removed. Both thresholds are updated live from `wall_detector_node`.

4. **DBSCAN (`clusterPointsAndBBoxes`).** Standard DBSCAN runs on the filtered cloud:
   - `dbscan_search_range_epsilon` — neighbourhood radius in world-frame metres
   - `dbscan_min_points_cluster` — minimum points to form a core
   Points not assigned to any cluster (noise) are discarded.

5. **Cluster refinement (optional, `visual_dbscan_refinement_enable`).** After DBSCAN, each raw cluster is processed by `refineClusterRecursive`:
   - A cluster is a **split candidate** when its XY diagonal > `dbscan_refine_max_diagonal` **and** its point density (pts / volume) < `dbscan_refine_min_density`. The diagonal is computed only in XY to avoid height variation triggering false splits.
   - The split is first attempted with a tighter DBSCAN pass (`dbscan_refine_split_eps`, `dbscan_refine_split_min_pts`). If that yields fewer than 2 sub-clusters, the algorithm falls back to **axis-slicing**: the longest axis is divided into slices of width `dbscan_refine_axis_slice_width`; contiguous non-empty slices are merged into sub-clusters.
   - Sub-clusters below `dbscan_refine_min_subcluster_pts` points are discarded. If `dbscan_refine_recursive: true`, each surviving sub-cluster is evaluated again recursively up to `dbscan_refine_max_depth` levels.
   - Sub-clusters whose bounding-box volume is below `dbscan_refine_min_box_volume` are clamped to that minimum when computing density (prevents division by near-zero for flat clusters).

   **Why refinement matters indoors.** In indoor environments the depth camera's ~5 m range covers the entire room, and objects like furniture, people near walls, or clustered items are spatially close. With a broad epsilon (needed to bridge depth noise), DBSCAN readily merges adjacent objects into one large, sparse super-cluster. Refinement detects these "merged blobs" — large diagonal + low density — and cleanly splits them before they enter the tracker. Outdoors, objects are generally well-separated and the depth range is shorter than the scene, so the merging problem is much rarer; refinement can be disabled (`visual_dbscan_refinement_enable: false`) to save CPU.

6. **Bounding box extraction.** For each final cluster, the AABB (min/max XYZ) is computed and stored as a `box3D`.

---

## 7. LiDAR detection (lidarDetect)

### Concept

The LiDAR provides a 360° scan that extends far beyond the depth camera's range (typically 20–30 m vs 5 m). The LiDAR pipeline runs in two stages: a **preprocessing callback** (`lidarOdomCB`) that fires on every synchronised (LiDAR, odom) pair and stores a clean world-frame cloud, and a **detection function** (`lidarDetect`) launched as one of the three parallel detector workers inside `mainCB()`.

### Preprocessing in `lidarOdomCB`

1. **XY range filtering.** A `pcl::PassThrough` filter retains only points within `±local_lidar_range.x` and `±local_lidar_range.y` of the sensor origin. This caps the computational load and discards background returns beyond the region of interest.

2. **Gaussian distance-based downsampling.** Each point surviving the XY filter is accepted with probability `p = exp(−dist² / (2σ²))` where σ = `gaussian_down_sample_rate` and `dist` is the planar distance to the sensor. This keeps a dense representation near the robot (where small/fast objects need precise boundaries) while thinning the far-field (where DBSCAN only needs to capture the cluster's bulk). Without this step, the near field would have orders of magnitude more points than the far field, making a single epsilon value poorly suited across the full range.

3. **World-frame transform.** The downsampled cloud is transformed from the LiDAR frame to the world frame using the currently buffered `orientationLidar_` and `positionLidar_`. The callback then refreshes those fields from the synchronized odometry/pose message near the end of the callback, so the transform applied to the cloud reflects the previous buffered LiDAR pose rather than a pose recomputed earlier in the same callback.

4. **Ground/roof and wall filtering.** A Z pass-through filter applies `[groundHeight_, roofHeight_]`, then `isInsideAnyWall` removes points inside known wall OBBs.

5. **Adaptive VoxelGrid downsampling.** A `pcl::VoxelGrid` is applied to the height- and wall-filtered cloud starting with a 0.1 m leaf size. If the point count still exceeds `downsample_threshold` (default 4000), the leaf size is multiplied by 1.1 and the filter is re-run, repeating until the count falls below the threshold. This is the most important computational safeguard in the preprocessing chain: it provides a hard upper bound on the number of points that DBSCAN will ever see, guaranteeing bounded execution time regardless of scene density. Without it, a dense near-range scan or a cluttered environment could inflate the cloud by an order of magnitude and stall real-time processing.

6. **Storage.** The resulting cloud is stored as `lidarCloud_` for the next `mainCB()` detection cycle.

### Detection in `lidarDetect`

1. **DBSCAN.** The stored `lidarCloud_` is clustered with `lidar_DBSCAN_epsilon` and `lidar_DBSCAN_min_points`. A larger epsilon than the visual DBSCAN is typical (e.g. 0.25 m vs 0.15 m) because spinning LiDARs have sparse angular sampling at range — consecutive scan lines can be centimetres apart on a 10 m object.

2. **Cluster refinement (optional, `dbscan_refinement_enable`).** The same `refineClusterRecursive` logic used by the visual pipeline is applied here. LiDAR shares the full set of refinement parameters with the visual path; the only difference is that visual DBSCAN has an independent override flag (`visual_dbscan_refinement_enable`) while LiDAR is gated solely by `dbscan_refinement_enable`. Indoors, LiDAR refinement is particularly effective: the 360° scan captures both sides of a doorway, a room corner, or two people walking together, and DBSCAN's epsilon easily bridges their gap. Refinement splits these merged blobs before they corrupt track geometry. Outdoors, objects are more spread out and the far-field angular sparsity naturally prevents merging, so refinement adds little benefit at the cost of more computation.

3. **Max-object-size filter.** After DBSCAN + refinement, each cluster whose bounding box exceeds `max_object_size [x, y, z]` in any axis is discarded. This removes walls, trees, and other large static structures that DBSCAN merges into single clusters before any further stage can handle them.

4. **Bounding box storage.** Surviving clusters and their AABBs are stored as `lidarBBoxes_` and `lidarClusters_` for the fusion stage.

LiDAR clusters are the backbone of the far-field detection pipeline. They are typically sparser than depth clusters but provide reliable coverage of vehicles and pedestrians well beyond the depth camera's range.

---

## 8. Fusion and YOLO association (filterLVBBoxes)

This function receives three independent lists of `box3D` (UV, DBSCAN, LiDAR) plus the latest YOLO 2D detections and produces a single fused, YOLO-annotated list.

The output of this stage is still a **detection list**, not a track list. IDs, velocities, acceleration, confirmation state, sticky YOLO evidence, and dynamic labels are assigned later by `kalmanFilterAndUpdateHist()` and the classification stage inside `mainCB()`.

### Local data-flow diagram

```
UV boxes ───────► intra-group cleanup ─┐
                                        ├─► inter-group UV/DBSCAN fusion ─► visual boxes ─┐
DBSCAN boxes ───► intra-group cleanup ─┘                                                   │
                                                                                            ├─► inter-group visual/LiDAR fusion
LiDAR boxes ────► intra-group cleanup ──────────────────────────────────────────────────────┘
                                                                                             │
                                                                                             ▼
                                                                                  optional final merge
                                                                                             │
                                                                                             ▼
                                                                                filteredBBoxesBeforeYolo_
                                                                                             │
YOLO 2D detections ─► depth ROI ellipse ─► foreground depth filter ─► 3D point cloud          │
                                                                                             ▼
                                                                  Case 1: depth points inside fused boxes
                                                                  Case 2: projected 3D box vs YOLO 2D IoU
                                                                                             │
                                                                                             ▼
                                                                                 filteredBBoxes_ + YOLO flags
```

The fusion steps do not require YOLO. YOLO is a semantic annotation layer applied after geometry has already produced candidate boxes.

### Phase A — Intra-group nesting cleanup

Before merging different sensors, `filterLVBBoxes()` first cleans duplicated/nested boxes **inside each single source**:

```cpp
mergeNestedGroup(uvBBoxes_)       → uvBBoxesFiltered
mergeNestedGroup(dbBBoxes_)       → dbBBoxesFiltered
mergeNestedGroup(lidarBBoxes_)    → lidarBBoxesFiltered
```

This is the **intra-group** logic: UV is compared only with UV, visual DBSCAN only with visual DBSCAN, and LiDAR only with LiDAR. It removes duplicates created by the same detector before any cross-sensor fusion happens.

`mergeNestedGroup()` has two stages:

1. **Mutual-IoU duplicate merge.** It computes the 3D IoU matrix among all boxes in the same group. A pair is merged only if each box is the other's best IoU match and both directions are above `samegroupIOU_threshold`. The output box is the geometric union of the two boxes, and the point clusters are concatenated.
2. **Nested-box graph merge.** On the remaining boxes, it computes directed IoV:

```cpp
IoV(A, B) = volume(intersection(A, B)) / volume(A)
```

If `IoV(A, B)` is high, A is mostly contained in B. The code creates a parent-child edge where the parent is the box that contains the other one. Each child keeps only its best parent, producing a small forest of nesting relationships:

```text
large parent box
        ├── smaller child box
        └── smaller child box
```

Each tree is then collapsed into one union box. Leftover boxes that were not part of any IoU or IoV relationship are passed through unchanged.

Important: `mergeNestedGroup()` does **not** use `visual_merging_flag` or `lidar_visual_merging_flag`. It always produces union boxes for same-source duplicates/nesting. The `smaller`/`bigger` policy belongs to the next function, `BboxesMerger()`.

### Phase B — Inter-group fusion

After intra-group cleanup, `filterLVBBoxes()` fuses boxes from different sources:

```cpp
BboxesMerger(uvBBoxesFiltered, dbBBoxesFiltered, ...)
    → visualBBoxesTemp

BboxesMerger(visualBBoxesTemp, lidarBBoxesFiltered, ...)
    → filteredBBoxesTemp
```

This is the **inter-group** logic: the first call fuses UV with visual DBSCAN, and the second call fuses visual evidence with LiDAR evidence.

#### Inter-group mutual IoU

`BboxesMerger()` first looks for one-to-one mutual best matches across the two groups. For example, in UV ↔ DBSCAN:

1. UV box `i` chooses the DBSCAN box with highest IoU.
2. That DBSCAN box must choose the same UV box as its highest-IoU partner.
3. Both scores must be above the configured IoU threshold.

When this happens, the output geometry is the union of the two boxes and the output point cluster is the concatenation of group-1 and group-2 clusters:

```cpp
cluster = group1_cluster + group2_cluster
```

The cluster center and standard deviation are then recomputed from the merged point set. For visual ↔ LiDAR mutual-IoU matches, the fused box therefore keeps both depth-camera points and LiDAR points instead of discarding one source.

The nested IoV branch below follows the same principle whenever it actually merges a parent/child pair or an entire subtree: the output cluster contains the points from all boxes that participate in the merge.

#### Inter-group nesting graph

For boxes not already consumed by mutual IoU, `BboxesMerger()` computes two directed IoV matrices:

```cpp
IOV_g1(i, j) = IoV(group1_box_i, group2_box_j)
IOV_g2(i, j) = IoV(group2_box_j, group1_box_i)
```

The larger directed IoV decides the containment direction:

- if `IOV_g1 > IOV_g2`, group1 box is mostly inside group2 box, so group2 becomes parent and group1 becomes child;
- otherwise group1 becomes parent and group2 becomes child.

Only edges above the configured IoV threshold are kept. As in the intra-group case, each child keeps its best parent, so a box cannot be nested under several parents at once. At this point all boxes participating in the graph are marked as used, and the selected `merging_style` decides what survives.

#### What `smaller` really does

`merging_style == "smaller"` does **not** compute a new mathematically smaller box. It keeps the **leaf nodes** of the nesting graph: the boxes that do not contain any other matched box.

Practical effect:

```text
large tree/vehicle/wall-like parent
        ├── small object candidate
        └── small object candidate

smaller → keep the small leaves, discard the large parent
```

This is useful when a large box contains one or more more-specific detections. It prevents the large container from replacing the smaller object boxes.

There is one conservative exception: if a leaf has exactly one parent and that parent has exactly one child, and `leaf_only == false`, the code outputs the union of parent and child. So in the simple one-parent/one-child case, `smaller` may still produce a merged union box. For the visual ↔ LiDAR stage this behavior is controlled by `lidar_visual_merger_leaf_only`; when it is true, the code keeps only the leaf and skips this conservative union.

In that conservative one-parent/one-child exception, the output cluster is a concatenation of the leaf cluster and the parent cluster. Therefore, for visual ↔ LiDAR nesting, this case can indeed contain both depth-camera points and LiDAR points.

#### What `bigger` really does

`merging_style == "bigger"` walks each nesting tree from its root and merges the whole subtree into one union box.

Practical effect:

```text
large parent
        ├── small child
        └── small child

bigger → one box covering parent + all children
```

This is useful when nested boxes are considered fragments of the same object and the desired output is a single conservative envelope. It is riskier around clutter, because a large static structure can absorb smaller valid detections.

Here the output cluster is also a concatenation of all point clusters in the subtree. For visual ↔ LiDAR nesting, `bigger` therefore can produce a mixed depth+LiDAR cluster.

#### Unmerged flags

Boxes that never participate in any inter-group IoU/IoV relationship are appended only if the corresponding flag is enabled:

- UV ↔ DBSCAN uses `uv_unmerged_flag` and `db_unmerged_flag`.
- visual ↔ LiDAR uses `visual_unmerged_flag` and `lidar_unmerged_flag`.

In outdoor mode `lidar_unmerged_flag: true` is critical because most objects beyond the depth range exist only as LiDAR clusters.

The current code only has explicit behavior for `merging_style == "smaller"` and `merging_style == "bigger"` in the nested-graph branch. Treat these as the supported values for `visual_merging_flag` and `lidar_visual_merging_flag`.

### Phase B.5 — Optional final merge

If `final_merge_flag: true`, the already fused `filteredBBoxesTemp` list is passed once more through `mergeNestedGroup()`. This is an **intra-group cleanup on the final fused list**, not another sensor-fusion step. It collapses residual duplicate/nested boxes before YOLO association, using `samegroupIOU_threshold` and `samegroupIOV_threshold`.

### Phase C — YOLO association

For each YOLO 2D detection whose class is in `yolo_dynamic_classes`:

#### STEP 1 — Build YOLO depth point cloud

The YOLO 2D bounding rect (in color image coordinates) is mapped to depth image coordinates. An **inscribed ellipse mask** is applied: pixels in the corners of the rectangle (outside the ellipse inscribed in the rect) are discarded. This is important because corners of YOLO rectangles often contain background pixels from adjacent objects, especially at object boundaries.

Valid depth pixels inside the ellipse are projected to 3D world-frame points → `yoloPoints`.

#### STEP 2 — Background filtering

The depth pixels inside the YOLO ellipse still contain background: ground, walls, parked vehicles behind the object, or terrain. The detector therefore performs a percentile-based foreground filter before attempting 3D association.

- **Indoor (`is_indoor: true`):** use the **10th-percentile depth** as the foreground estimate and keep only points within `yolo_depth_tolerance` meters behind it. This is intentionally tight because indoor YOLO boxes often overlap walls immediately behind the object.
- **Outdoor (`is_indoor: false`):** use the **20th-percentile depth** and `yolo_outdoor_depth_tolerance`. Outdoor objects, especially cars and cyclists seen at an angle, can span a deeper range than indoor people, so the tolerance is wider.
- Both modes apply the live height bounds `[ground_height, roof_height]`.

Result: `filteredYoloPoints` — a tight 3D point cloud representing the foreground object seen by YOLO.

After each `filterLVBBoxes()` pass, the raw YOLO detection array is cleared. The long-lived semantic memory is not the raw YOLO message; it is the sticky `is_yolo_candidate` flag propagated later in track history by `kalmanFilterAndUpdateHist()`. This prevents one YOLO message from being re-applied across multiple detector cycles while still allowing a confirmed YOLO track to survive frames where YOLO has not published a fresh detection.

#### STEP 3A — Point-based match with YOLO depth points

For each fused 3D box, count how many `filteredYoloPoints` fall inside it. A box can become a YOLO candidate only if:

```
inside_count / total_filtered_yolo_points >= yolo_point_fraction_threshold
```

This is a **many-to-one** association: multiple 3D boxes are allowed to match the same YOLO 2D detection. This is required because a single person/car YOLO rectangle can cover several separate 3D clusters, especially when LiDAR and depth split the object or when a partially occluded object is segmented into pieces.

##### Sparse-large rejection

A large 3D box that contains only a few YOLO points is a common false-positive pattern: the real YOLO object sits near or inside a large tree/wall/vehicle box, and a small number of foreground points happens to fall inside that large box.

To prevent this, the association has an additional gate for boxes with volume ≥ `yolo_sparse_large_box_volume_threshold`:

```
sparse_large_box =
    box_volume >= yolo_sparse_large_box_volume_threshold
    AND (
        yolo_inside_count < yolo_sparse_large_min_points
        OR yolo_inside_count / box_volume < yolo_sparse_large_min_density
    )
```

If `sparse_large_box` is true, the box is **not** marked as `is_yolo_candidate`, even if the fraction gate passes. This keeps association permissive for normal/small boxes while making it harder for huge sparse boxes to inherit a YOLO dynamic label.

##### Height-ratio rejection

The detector also compares the 3D box height against the vertical span of the YOLO foreground points inside the box:

```
yolo_z_span = max(z_inside) - min(z_inside)
yolo_z_span = max(yolo_z_span, yolo_min_height_span)
box_too_tall = box.z_width > yolo_z_span * yolo_max_height_ratio
```

If the box is much taller than the actual YOLO foreground, it is rejected. This protects against a person/car detection partially overlapping a tall static structure.

**Height correction:** if the Z range of inside points differs from the box's Z width by ≥ `yolo_height_correction_threshold`, the box height and Z center are overridden with the YOLO-derived values. YOLO gives an accurate silhouette of the object which often constrains height better than LiDAR clusters (which over-extend vertically due to ground reflections or multi-layer scan lines).

**XY resize (`yolo_x_y_resize: true`):** the box X/Y extents are also shrunk to fit the inside points, and the cluster is re-filtered using only those points. This tightens the bounding box to the true object footprint as seen by depth, at the cost of some robustness to depth holes.

In code, the point-based decision is:

```
point_based_candidate =
    yolo_inside_count > 0
    AND
    fraction >= yolo_point_fraction_threshold
    AND NOT sparse_large_box
    AND NOT box_too_tall
```

#### STEP 3B — 2D IoU fallback

The 2D IoU fallback is used when depth evidence is missing or too weak to associate a YOLO detection with a specific 3D box.

For each candidate 3D box, the detector projects the 8 bbox corners into the color image using the color-camera pose/intrinsics. It then builds the axis-aligned 2D rectangle that contains those projected corners and computes its 2D IoU against the YOLO rectangle. If the IoU is at least `yolo_2d_iou_threshold`, the box can be marked as `is_yolo_candidate`.

There are two fallback paths:

1. **Global no-depth fallback.** If `yoloPoints` is empty, or if all YOLO depth points are removed by foreground/height filtering and `filteredYoloPoints` is empty, the detector runs `tryIouFallback()` over every fused 3D box. In this mode, any box whose projected 2D rectangle overlaps the YOLO rectangle enough becomes `is_yolo_candidate`.
2. **Per-box sparse-depth fallback.** If the YOLO detection has usable depth points, each box first tries the point-based association above. If that fails because this specific box has sparse depth support, the same projected-2D IoU test is attempted for that box.

```cpp
point_based_candidate =
    yolo_inside_count > 0
    AND fraction >= yolo_point_fraction_threshold
    AND NOT sparse_large_box
    AND NOT box_too_tall

sparse_depth_support =
    yolo_inside_count < yolo_sparse_large_min_points
    OR fraction < yolo_point_fraction_threshold

iou_fallback_candidate =
    NOT point_based_candidate
    AND sparse_depth_support
    AND projected_3d_box_2d_iou >= yolo_2d_iou_threshold
```

This covers the mixed case where a YOLO detection partially has valid depth returns, but the relevant LiDAR box is beyond the useful depth range or receives only a few depth pixels. In that situation the box can still become `is_yolo_candidate` from camera projection overlap.

Important implementation detail: the `sparse_large_box` and `box_too_tall` gates currently belong to the **point-based** association path. They are not direct hard gates on the 2D fallback, but fallback is attempted only when this specific box has sparse depth support (`inside_count < yolo_sparse_large_min_points` or fraction below threshold). Therefore, for LiDAR-only/no-depth associations, `yolo_2d_iou_threshold` is the main strictness control; for boxes with many YOLO depth points but rejected by shape gates, the code generally does not use 2D IoU as a second chance.

When a box is accepted only by 2D IoU fallback, the detector does **not** apply YOLO-based X/Y resize or height correction, because those corrections require reliable inside-depth points.

### What YOLO association does not do

YOLO association does **not** immediately make a track dynamic. It only sets `is_yolo_candidate` on the detection/track. The later classification stage decides:

- moving YOLO candidate → `dynamic`
- stationary YOLO candidate → `potentially_dynamic`

This distinction is important for downstream consumers such as GLIM and Nav2: a parked car or standing person is a dynamic-class object, but not necessarily moving right now.

---

## 9. Kalman filter and track management

### Motion model

The active tracker uses `kalmanFilterMatrixAccV2()`, a **2D constant-acceleration** Kalman filter. The state is 6-dimensional:

```
x = [px, py, vx, vy, ax, ay]ᵀ
```

The state transition matrix `A` is parametrized by `dt` (the time elapsed since the last frame):

```
A =
[ 1  0  dt  0  0.5dt²   0    ]
[ 0  1  0   dt   0    0.5dt² ]
[ 0  0  1   0    dt     0    ]
[ 0  0  0   1    0      dt   ]
[ 0  0  0   0    1      0    ]
[ 0  0  0   0    0      1    ]
```

Only XY position is observed directly:

```
z = [measured_x, measured_y]ᵀ
H = [1 0 0 0 0 0
     0 1 0 0 0 0]
```

Z is not part of the Kalman state. On matched detections, `newEstimatedBBox.z` is copied from the current detected box, while XY velocity and acceleration are estimated by the filter from the sequence of XY position observations. The published/tracked bbox dimensions are also taken from the current detection rather than predicted by the filter.

**Noise parameters** (`kalman_filter_v2_param: [eP, eQPos, eQVel, eQAcc, eRPos]`):
- `eP` — initial state covariance (uncertainty at track creation).
- `eQPos/eQVel/eQAcc` — process noise on position/velocity/acceleration. Higher `eQVel`/`eQAcc` makes the filter respond faster to acceleration changes (appropriate for cars).
- `eRPos` — measurement noise on position. Higher values make the filter trust measurements less and rely more on the prediction.

### Association scoring

Each tracking cycle first predicts each existing track's next XY position. Incoming detections are then matched to those predictions using a composite score. In the current implementation, **higher score is better**:

```
score =
    - match_pos_score_weight          * normalized_distance(predicted, detection)
    - match_size_score_weight         * relative_size_diff
    + match_iou2d_score_weight        * IoU2D(predicted, detection)
    - match_prev_obs_pos_score_weight * normalized_distance(previous_observation, detection)
    + match_prev_obs_iou2d_weight     * IoU2D(previous_observation, detection)
    - match_velocity_direction_weight * velocity_direction_error
    - match_yolo_class_weight         * yolo_missing_penalty
    + confirmed_track_assoc_bonus       if the previous track is confirmed
    + yolo_track_assoc_bonus            if the current detection is YOLO-associated
    + outside-FOV association bonuses   for LiDAR-only outside-FOV matches
```

The association pipeline has three layers:

1. **Hard physical gates.** Reject impossible matches before scoring: too far, too fast, too different in size, inconsistent velocity direction, or unnatural motion.
2. **Score ranking.** For every remaining pair `(current_detection, previous_track)`, compute the score above. Better geometric continuity and helpful semantic evidence push the score upward.
3. **Minimum-score rejection and global assignment.** A candidate is usable only if it reaches its adaptive minimum score. Then the assignment step chooses a one-to-one set of matches.

Candidates with score below the adaptive minimum are rejected:

- `min_match_score` for normal tentative tracks.
- `min_match_score_confirmed` for confirmed tracks.
- `min_match_score_dynamic` when the current detection is YOLO-associated.

Example:

```text
min_match_score = -2.5
score = -1.0  → accepted candidate
score = -3.0  → rejected candidate
```

The score is therefore a mixture of penalties and bonuses. Distances, size mismatch, velocity-direction disagreement, and missing YOLO evidence inside the FOV reduce the score. IoU, confirmation history, YOLO evidence, and outside-FOV association bonuses raise it. More permissive minimum scores are more negative; stricter minimum scores are closer to zero or positive.

The adaptive threshold is selected from the current detection/track context:

```cpp
if current_detection.is_yolo_candidate:
    adaptiveMinScore = min_match_score_dynamic
else if previous_track_is_confirmed:
    adaptiveMinScore = min_match_score_confirmed
else:
    adaptiveMinScore = min_match_score
```

Because the YOLO check comes first in the code, a YOLO-associated detection uses `min_match_score_dynamic` even if the previous track is already confirmed.

The velocity direction error term compares the observed velocity from the previous observation to the current detection against the Kalman-predicted velocity. It includes both velocity-vector difference and direction mismatch, and it returns zero when both observed and predicted motion are below `stationary_speed_thresh`.

A **speed gate** additionally rejects assignments where the implied speed exceeds `max_match_speed`; LiDAR-only outside-FOV associations can use `max_match_speed_outside_fov`.

### Association rejection gates

Before a candidate match is scored, hard gates can reject it:

- **Position gate:** predicted center vs detected center must be within `max_match_range`; outside camera FOV the relaxed `max_match_range_outside_fov` can be used.
- **Implied speed gate:** center displacement divided by `dt` must be below `max_match_speed`; outside FOV uses `max_match_speed_outside_fov`.
- **Relative size gate:** box dimensions must be compatible. Confirmed tracks get a small tolerance relaxation, and LiDAR-only outside-FOV associations can use `max_relative_size_diff_outside_fov`.
- **Velocity-direction gate:** the current detection must be compatible with the predicted track motion. YOLO-associated detections or tracks with sticky YOLO evidence use the more permissive dynamic direction thresholds, not the strict non-YOLO thresholds.
- **Abrupt-turn exception:** mature confirmed tracks outside the FOV can survive a large heading change if their position, speed, observed speed, and size are within `outside_fov_turn_*` limits.
- **Natural-motion gate:** unconfirmed non-YOLO tracks must pass `isNaturalMotion()`. Inside FOV this rejects too-small motion when `track_steady_objects: false`; everywhere it rejects excessive jumps and large Kalman innovation.

Candidate assignment is global enough to avoid assigning the same previous track to multiple current detections. Valid candidates are sorted by score from best to worst, then `assignMatchesGlobally()` builds a one-to-one assignment with an augmenting-path pass over the ordered candidate lists.

### Track lifecycle

```
                 new detection
                      │
                      ▼
              UNCONFIRMED TRACK
                      │
                      │ enough valid hits
                      │ or fresh YOLO evidence
                      ▼
                CONFIRMED TRACK
                      │
       ┌──────────────┴────────────────┐
       │                               │
 matched this frame              missed this frame
 reset missed counter            Kalman predict only
       │                               │
       ▼                               ▼
 output if not suppressed         COASTING TRACK
                                       │
                                       │ misses exceed lifetime
                                       │ (`max_missed_frames`, or
                                       │  `max_missed_frames_yolo`)
                                       ▼
                                    DELETED
```

Confirmation is asymmetric:

- A new unmatched YOLO-associated detection is confirmed immediately and can enter `trackedBBoxes_` on its first update, except during the very first tracker initialization pass when `boxHist_` is empty and all initial detections are only used to seed histories and filters.
- A matched track with current or historical YOLO evidence is also confirmed immediately by `shouldConfirmTrack()`.
- A non-YOLO track inside the FOV needs `min_confirm_hits`, non-stationary observed motion when `track_steady_objects: false`, and a valid velocity-direction gate.
- A non-YOLO track outside the FOV uses the stricter outside-FOV bootstrap: `min_confirm_hits_outside_fov`, observed average speed inside `[min_outside_fov_avg_obs_speed, max_outside_fov_avg_obs_speed]`, natural motion, and velocity-direction consistency.

Unmatched tracks are propagated internally by Kalman prediction and missed-frame counters, but predicted-only boxes are **not** pushed to `trackedBBoxes_`. Output only contains current detections that were matched to confirmed tracks, plus newly created YOLO-confirmed tracks.

### Confirmation and output suppression

Track confirmation and track publication are related but not identical:

- Once confirmed, a track remains confirmed until it dies by missed-frame timeout.
- A confirmed non-YOLO track whose KF speed falls below `stationary_speed_thresh` can be kept internally but suppressed from `trackedBBoxes_` when `track_steady_objects: false`.
- YOLO-candidate tracks are still published when stationary, because a stationary person or parked car is still a semantically dynamic-class object and may need to be represented downstream as `potentially_dynamic`.

This design avoids flooding the navigation stack with stationary non-semantic clutter while preserving people/vehicles even when they stop.

There is one snapshot detail to remember: inside `mainCB()`, `trackedBBoxes_` is assembled in `kalmanFilterAndUpdateHist()` before the classification stage runs. For matched confirmed tracks, that first output snapshot may initially inherit `prevDynamic` and `prevPotentiallyDynamic` from the previous history entry.

The current implementation fixes this before visualization/publication: after classification fills `dynamicBBoxes_` and `potentiallyDynamicBBoxes_`, `mainCB()` iterates again over `trackedBBoxes_` and refreshes each tracked output box from the latest `boxHist_[i][0].is_dynamic` / `is_potentially_dynamic` values using the track id.

Practical consequence: `/tracked_bboxes` text/color and `ObstacleArray` status use the current-cycle classification flags, not a one-cycle-old snapshot. The geometry, velocity, age, and track id in `trackedBBoxes_` still come from the tracking output; only the semantic classification flags are patched after the classification stage. Without this final refresh, tracked-box text/color and `ObstacleArray` status could lag one `mainCB()` cycle behind.

One nuance: when `target_constrain_size` is enabled, the dedicated `/dynamic_bboxes` list can be filtered after a track is marked dynamic in `boxHist_`. In that case, `trackedBBoxes_` and `ObstacleArray` may still show the track as dynamic while the dedicated dynamic marker list omits it. That is a size-filtering effect, not a stale-snapshot effect. `/potentially_dynamic_bboxes` does not go through that size filter.

### Sticky YOLO evidence

`is_yolo_candidate` is propagated at the track level:

```
new_estimated_box.is_yolo_candidate =
    current_detection.is_yolo_candidate OR previous_track.is_yolo_candidate
```

It is cleared by two guards:

1. **Inside-FOV no-YOLO decay:** if a track is inside the camera FOV for `max_non_yolo_in_fov_frames` consecutive frames without a fresh YOLO hit, the sticky flag is cleared.
2. **Outside-FOV base-size growth guard:** when a track leaves the FOV, LiDAR-only boxes may shrink naturally. Shrinkage is ignored. Growth beyond the stored YOLO baseline for `max_yolo_base_mismatch_frames` frames clears the YOLO flag, because it suggests an ID swap toward a larger object.

The stored YOLO baseline uses the maximum X/Y size ever seen during fresh YOLO detections, preventing temporary occlusions from shrinking the baseline and causing false mismatch.

### Duplicate suppression

The tracker has two duplicate filters for new unmatched detections:

- `isCloseToExistingTrack()` compares a new detection against every predicted existing track using center distance, 2D IoU, and relative size difference. If it is too similar to an existing prediction, no new track is created.
- `isDuplicateOfTrackedThisFrame()` compares the new detection against boxes already emitted in the current tracking cycle. This prevents two detections in the same frame from spawning duplicate outputs for the same object.

---

## 10. Dynamic classification stage

The classification stage runs near the end of `mainCB()`, after fusion, association, and Kalman tracking have already produced `trackedBBoxes_`. It does not classify every internal track in `boxHist_`: first it builds the set of track ids currently present in `trackedBBoxes_`, then it processes only those tracks. This means predicted-only missed tracks, unconfirmed tracks, and stationary non-YOLO tracks suppressed from output are kept alive internally but are skipped by classification for that cycle.

The classifier writes the latest semantic state into `boxHist_[i][0]`:

- `is_dynamic = true` means the object is considered moving now.
- `is_potentially_dynamic = true` means YOLO recognized a dynamic-class object, but the current KF speed is below the dynamic threshold.
- `is_dynamic_candidate = true` means the current frame had motion evidence, but the track has not necessarily passed the multi-frame consistency gate yet.

The final published arrays are then built from that state: `dynamicBBoxes_` receives moving dynamic tracks, `potentiallyDynamicBBoxes_` receives stationary YOLO dynamic-class tracks, and `trackedBBoxes_` is refreshed with the same flags before visualization and `ObstacleArray` publication.

The paths are evaluated in order. PATH 1 has priority over everything else, PATH 2 is a shortcut for tracks that were already dynamic recently, and PATH 3 is the full point-cloud/KF evidence path.

```
confirmed published track
        │
        ├─ is_yolo_candidate?
        │       ├─ speed >= dynamic_velocity_threshold → dynamic
        │       └─ speed <  dynamic_velocity_threshold → potentially_dynamic
        │
        ├─ enough recent dynamic frames?
        │       ├─ speed still high
        │       ├─ outside-FOV non-YOLO observed motion ok, if needed
        │       └─ yes → dynamic
        │
        └─ full motion-evidence path
                ├─ point-cloud nearest-neighbor votes
                ├─ OR confident KF speed
                ├─ AND speed hard gate
                ├─ AND outside-FOV observed motion ok, if needed
                └─ consistency threshold → dynamic
```

There are therefore two different questions being answered:

1. **Can this object belong to a dynamic semantic class?** YOLO answers this through `is_yolo_candidate`.
2. **Is it moving right now?** KF speed, point-cloud motion, and outside-FOV observed motion answer this.

This separation is why a standing person and a parked car are not called `dynamic`; they are `potentially_dynamic`.

### PATH 1 — YOLO candidate

A track is on PATH 1 if its `is_yolo_candidate` flag was set during YOLO association in the current or a recent frame (the flag is sticky for `max_non_yolo_in_fov_frames` frames inside the FOV, longer outside it).

Classification is purely speed-based using the Kalman-estimated velocity:

- `‖v‖ ≥ dynamic_velocity_threshold` → **dynamic** (`is_dynamic = true`). The track is flagged as a confirmed moving obstacle and published on `/dynamic_bboxes`.
- `‖v‖ < dynamic_velocity_threshold` → **potentially dynamic** (`is_potentially_dynamic = true`). The object is recognized by YOLO as a dynamic class (e.g., a parked car or a standing person) but is currently stationary. Published on `/potentially_dynamic_bboxes`.

YOLO-candidate tracks bypass motion voting entirely. This prevents a stationary person from being wrongly classified as a static obstacle just because the point cloud isn't moving.

The practical interpretation is:

- **YOLO tells the system the object class can move.**
- **The Kalman velocity tells the system whether it is moving now.**

This is why a parked car or standing person appears as `potentially_dynamic`, not `dynamic`.

### PATH 2 — Historical continuity

PATH 2 exists to avoid re-running the full point-cloud motion test on every frame for an object that was already classified dynamic very recently. The code looks at previous history entries, starting from `boxHist_[i][1]`, and counts how many of the last `frames_force_dynamic_check_range` frames had `is_dynamic = true`. If that count is at least `frames_force_dynamic`, the track can be kept dynamic immediately.

This is still gated by current motion. The current KF speed must be ≥ `dynamic_velocity_threshold`. Therefore a track that stops moving loses `dynamic`; the recent dynamic history alone is not enough.

For non-YOLO tracks outside the camera FOV, this shortcut is accepted only if the outside-FOV observed-motion test also passes. This prevents a noisy static LiDAR cluster from remaining dynamic merely because the Kalman filter still has residual velocity.

### PATH 3 — Point-cloud motion voting

Tracks that are neither YOLO candidates nor already sustained by historical continuity go through the full motion-evidence path. This path combines local point-cloud motion with the Kalman velocity estimate.

1. **Choose the temporal baseline.** The current point cloud is compared with the cloud from `frame_skip` frames ago. If the track is younger than that, the code uses the oldest available history entry.
2. **Estimate box motion.** `Vbox` is computed from the centroid displacement between the current bbox and the historical bbox. `Vkf` is the Kalman velocity estimate stored in `boxHist_[i][0]`.
3. **Compute per-point motion.** Each current point finds its nearest neighbour in the historical cloud. The displacement divided by `dt * frame_gap` gives a per-point velocity `Vcur`.
4. **Vote only with compatible points.** If the per-point velocity points opposite to `Vbox`, that point is removed from the effective denominator. Otherwise, it votes dynamic when `|Vcur| > dynamic_velocity_threshold`.
5. **Build one-frame evidence.** The frame has dynamic evidence if the KF speed hard gate passes and at least one soft cue is true: `voteRatio >= dynamic_voting_threshold`, or the KF velocity is confident.
6. **Apply outside-FOV protection.** For outside-FOV non-YOLO tracks, the observed-motion gate must also pass. This stops static LiDAR clusters with jittery KF velocity from becoming dynamic.
7. **Apply consistency.** Single-frame evidence sets `is_dynamic_candidate = true`. The track becomes `is_dynamic = true` only when all of the last `dynamic_consistency_threshold` history entries contain evidence (`is_dynamic_candidate`, `is_yolo_candidate`, or `is_dynamic`).

The Kalman-confidence cue is meant for sparse clouds, especially outside the depth camera range. It computes the recent KF speed mean and standard deviation. It is true when the mean speed is above `dynamic_velocity_threshold` and the standard deviation is small relative to the mean (`std <= dynamic_kf_vel_std_ratio * mean`). This can replace point-vote evidence, but it does not bypass the final KF speed hard gate.

The consistency gate is stricter than a single-frame vote. A noisy frame can mark the track as `is_dynamic_candidate`, but it will not publish as `dynamic` until enough consecutive history entries show evidence.

### Outside camera FOV

For non-YOLO tracks that have moved out of the camera FOV, depth-based motion voting is unreliable because the current cluster is usually LiDAR-only and sparse. The classifier therefore adds an observed-motion gate based only on the sequence of associated bbox centers stored in `boxHist_`.

This check is implemented by `hasConsistentObservedMotionOutsideFov()`:

1. **Choose the history window.** The code compares the current box center, `boxHist_[i][0]`, with an older center, `boxHist_[i][k]`, where `k = min(outside_fov_class_window, history_size - 1)`. The window is therefore as long as configured when enough history exists, and shorter for young tracks.
2. **Compute net motion.** `netDisp` is the straight-line XY displacement between the oldest and newest centers in the window. It is not the sum of all small frame-to-frame moves; it answers: "after this many frames, how far did the object actually move away from where it was?"
3. **Compute net speed.** `netSpeed = netDisp / (k * dt)`. This is an average speed over the whole window, based on the endpoint displacement.
4. **Compute travelled path and straightness.** `pathLen` is the sum of every frame-to-frame XY displacement inside the same window. `straightness = netDisp / pathLen`:
   - close to `1.0`: the centers move coherently in one direction;
   - much lower than `1.0`: the centers move back and forth, curve sharply, or jitter around nearly the same place.
5. **Reject impossible jumps.** `maxStepSpeed` is the largest single frame-to-frame speed inside the window. If it is too high, the motion is treated as a likely association jump or cluster glitch.
6. **Conditions for dynamic evidence outside FOV:**
   - Net speed ≥ `outside_fov_class_min_net_speed`
   - Net displacement ≥ `outside_fov_class_min_net_disp`
   - Straightness ≥ `outside_fov_class_min_straightness`
   - Per-step speed ≤ `outside_fov_class_max_step_speed` (eliminates spurious jumps)
   - Track age ≥ `outside_fov_class_min_track_age`

The important distinction is that this gate is measuring **observed displacement of the matched boxes**, not just Kalman velocity. A static object with noisy LiDAR boxes may produce non-zero instantaneous velocity, but it normally has small net displacement, low straightness, or one large suspicious step. That is why this test is required before a non-YOLO outside-FOV track can remain dynamic through historical continuity or become dynamic through new point-cloud/KF evidence.

Objects that genuinely change direction are handled in two places:

- **Association stage:** before classification, `computeAssociationScore()` can accept an abrupt outside-FOV turn through `isAbruptTurnAllowedOutsideFov()` when the track is mature, non-YOLO, still close enough to the prediction, moving within configured speed limits, size-compatible, and has coherent observed motion. This is the mechanism that prevents a valid turning object from immediately losing its track ID.
- **Classification stage:** after association, the same observed-motion gate remains conservative. If the history window spans a very sharp turn, `straightness` can drop because `pathLen` becomes much larger than the endpoint displacement. In that case the object can stay associated, but dynamic classification may be delayed until the recent window again shows enough coherent net motion. Tune `outside_fov_class_window` and `outside_fov_class_min_straightness` if valid turns are being suppressed too long.

For outside-FOV **YOLO** tracks, PATH 1 still applies: moving YOLO tracks become dynamic, stationary YOLO tracks remain potentially dynamic. The outside-FOV observed-motion gate described here is only added for non-YOLO tracks.

### Optional dynamic-size filter

After classification, if `target_constrain_size` is enabled, only `dynamicBBoxes_` is post-filtered by size. A dynamic box is kept in `/dynamic_bboxes` if it is close to at least one configured `target_object_size` within fixed tolerances (`|dx| < 0.8`, `|dy| < 0.8`, `|dz| < 1.0`). This filter does not change `boxHist_` and does not apply to `potentiallyDynamicBBoxes_`.

This distinction matters during debugging: a track can be classified dynamic in `boxHist_` and appear as dynamic in `trackedBBoxes_` / `ObstacleArray`, but be missing from `/dynamic_bboxes` if the optional size filter removes it from the dedicated dynamic marker list.

---

## 11. Outside-FOV association and classification

When a non-YOLO track and the current detection are both outside the camera FOV, normal association may fail because the box shapes differ between LiDAR-only and LiDAR+depth detection modes. The current implementation does not run a separate second association pass; instead, `computeAssociationScore()` switches to outside-FOV gates and bonuses when `isLidarOnlyOutsideFovAssociation()` is true.

- **Position gate:** `max_match_range_outside_fov`. This is an outside-FOV-specific limit, not necessarily wider than the in-FOV `max_match_range`; in the current outdoor config it is intentionally tighter in position while allowing high speed.
- **Speed gate:** `max_match_speed_outside_fov`. This can be higher outdoors to allow fast cars without opening the positional gate too much.
- **Size tolerance:** `max_relative_size_diff_outside_fov`. LiDAR-only clusters are typically smaller or less stable than their LiDAR+depth equivalents, so a dedicated size limit is used for this case.
- **Innovation gate:** `max_natural_innovation_outside_fov`. This is used by the natural-motion gate for outside-FOV tentative tracks.
- **Abrupt-turn association:** normal direction checks can reject a detection when the measured displacement points in a very different direction from the track velocity. For confirmed, mature, non-YOLO tracks outside the FOV, `isAbruptTurnAllowedOutsideFov()` can override that rejection. The detection is accepted only if both predicted and current boxes are outside the camera FOV, the current box remains within `outside_fov_turn_max_pos_dist` from the prediction, observed speed is inside [`outside_fov_turn_min_obs_speed`, `outside_fov_turn_max_speed`], relative size difference is below `outside_fov_turn_max_size_diff`, and the observed-motion gate still passes. When accepted, the score receives `outside_fov_turn_assoc_bonus`. This keeps the same track alive through realistic turns, but it does not by itself force the track to become dynamic; classification still applies the outside-FOV dynamic gates described above.

**YOLO ID-swap guard:** after a YOLO detection on the object, the `yolo_base_size` is recorded. If the track's bounding box grows by more than `yolo_base_mismatch_thresh` (e.g., 150%) relative to that baseline for `max_yolo_base_mismatch_frames` consecutive frames, the `is_yolo_candidate` flag is cleared. This prevents LiDAR noise or a nearby wall from inheriting a pedestrian's YOLO label after an ID swap outside the FOV.

---

## 12. yolov11_detector_node

A pure-Python ROS 2 node that runs YOLOv11 inference using the `ultralytics` library. It subscribes to the color image topic, runs inference at `timer_period_sec` intervals (~30 Hz), and publishes a `vision_msgs/Detection2DArray` with 2D bounding boxes and class labels.

Only classes listed in `target_classes` are forwarded. All other detections are silently discarded before publishing to avoid loading the `detector_node` association step with irrelevant boxes (e.g., traffic lights, furniture).

The inference size (`inference_size: 640`) controls the resolution at which YOLO processes the image. Higher values improve detection at long range (better for cars) at the cost of latency.

---

## 13. odometry_tf_publisher_node

A thin C++ node that subscribes to `/odometry/filtered` (EKF output from `robot_localization`) and re-broadcasts it as a `odom → base_link` TF transform at the same timestamp. This is only needed when running the detector **without** the full `jo_navigation` stack (e.g., playing a bag without starting EKF). When EKF is running, it publishes the TF itself and this node is redundant (disable with `odom_pub:=false`, which is the default).

---

## 14. ROS outputs and status semantics

`visCB()` publishes both visualization topics and machine-readable obstacle messages. The most important output for downstream autonomy is:

```
/perception/tracked_dynamic_obstacles
```

Its type is `jo_msgs/ObstacleArray`. Despite the topic name, it does **not** publish every confirmed track: purely static tracks are skipped. Each published element contains:

- `track_id` — persistent ID assigned by the tracker.
- `track_age` — number of frames the track has been alive.
- `status` — semantic/dynamic state.
- `pose` — current tracked bbox center.
- `size` — current tracked bbox dimensions.
- `twist` — Kalman-estimated linear velocity.
- `accel` — Kalman-estimated acceleration.

### Obstacle status values

| Status | Meaning | Typical source |
|---|---|---|
| `STATUS_STATIC = 0` | Confirmed track with no current dynamic evidence | Mature non-YOLO object or stopped non-semantic track |
| `STATUS_DYNAMIC = 1` | Track currently classified as moving | YOLO + speed, historical dynamic continuity, or point-cloud/KF motion evidence |
| `STATUS_POTENTIALLY_DYNAMIC = 2` | YOLO dynamic-class object that is currently stationary | Standing person, parked car, stopped bicycle |
| `STATUS_TRACKED = 3` | Confirmed/tracked object moving above the KF speed threshold but not yet fully classified dynamic | Transition state useful for consumers that want early caution |

`STATUS_STATIC` exists in the message definition for completeness, but `publishDynamicObstacleArray()` currently skips static tracks:

```
if dynamic              → publish STATUS_DYNAMIC
else if potentially     → publish STATUS_POTENTIALLY_DYNAMIC
else if speed >= thresh → publish STATUS_TRACKED
else                    → skip
```

The distinction between `STATUS_DYNAMIC` and `STATUS_POTENTIALLY_DYNAMIC` is intentional. A person standing still should not be painted as an actively moving obstacle, but downstream systems may still want to preserve it, inflate it, or treat it differently from static geometry.

### Marker color convention

The 3D box markers force color from the semantic flags:

- `is_dynamic` → blue.
- `is_potentially_dynamic` → green.
- otherwise → caller-provided color.

This avoids the old ambiguity where a label could say `dynamic` while the marker color came from the generic tracked-box color.

### Main visualization topics

The exact namespace depends on `detector_namespace`. With an empty namespace the publishers use the root topic names below; with `detector_namespace: jo_detector`, the same names are prefixed with `/jo_detector`.

| Topic | Content |
|---|---|
| `/filtered_before_yolo_bboxes` | Fused boxes before YOLO annotation |
| `/filtered_bboxes` | Final detection boxes after YOLO annotation, before tracking |
| `/tracked_bboxes` | Confirmed tracked boxes |
| `/predicted_bboxes_active` | Predictions for active tracks |
| `/predicted_bboxes_missed` | Predictions for temporarily missed tracks |
| `/predicted_bboxes_unconfirmed` | Predictions for tentative/unconfirmed tracks |
| `/dynamic_bboxes` | Boxes currently classified dynamic after optional `target_constrain_size` filtering |
| `/potentially_dynamic_bboxes` | YOLO dynamic-class boxes currently stationary |
| `/yolo_points` | YOLO foreground depth points used for association |
| `/dynamic_point_cloud` | Points inside current dynamic boxes |
| `/raw_dynamic_point_cloud` | Raw latest LiDAR points that fall inside current dynamic boxes |
| `/filtered_lidar_points_velodyne_frame` | LiDAR points after XY range filtering and Gaussian distance downsampling, still expressed in the Velodyne frame; despite the topic name, this is not GLIM-style dynamic-obstacle filtering |
| `/downsampled_point_cloud` | LiDAR cloud after world transform, ground/roof and wall filtering, and adaptive voxel downsampling |
| `/raw_lidar_point_cloud` | Latest raw LiDAR cloud transformed to `odom` for debug visualization |
| `/detected_color_image` | Color image annotated with YOLO rectangles |
| `/history_trajectories` | Track trajectory history markers |
| `/velocity_visualizaton` | Velocity marker visualization, keeping the existing topic spelling |

These topics are for debugging and RViz interpretation. The stable machine interface should remain `ObstacleArray`.

### Interaction with GLIM and Nav2

GLIM subscribes to `/perception/tracked_dynamic_obstacles` in BBOX mode. It can remove raw LiDAR points using the bbox status, track age, velocity, and configured rejection geometry.

Nav2 can consume the same obstacle stream through the custom dynamic obstacle layer. That layer builds 2D risk regions in the costmap using the obstacle velocity and status. Because both GLIM and Nav2 consume the same high-level message, the semantics above must remain consistent: `dynamic` means moving now; `potentially_dynamic` means semantically dynamic but currently stationary.

---

## 15. Indoor vs outdoor mode

The `is_indoor` flag in the config file controls several pipeline behaviours simultaneously:

| Behaviour | Indoor (`true`) | Outdoor (`false`) |
|---|---|---|
| **Depth background filter in YOLO association** | 10th-percentile foreground depth + `yolo_depth_tolerance` | 20th-percentile foreground depth + `yolo_outdoor_depth_tolerance` |
| **ICP calibration timeout** | No timeout — node waits indefinitely for `icp_max_runs` valid scenes | After `icp_timeout_sec` seconds, publishes partial average or initial guess |
| **Typical YOLO association mode** | Case 1 (depth available) dominant — depth camera fully covers scene | Case 2 (IoU fallback) common — many objects beyond depth range |
| **Recommended `icp_fitness_threshold`** | 0.5 (strict — indoor geometry constrains ICP well) | 0.15 (relaxed — fewer planar surfaces) |
| **Detection range** | `local_sensor_range: [5, 5, 5]` m — tight, no far-field | `local_lidar_range: [15, 15, 5]` m — wide, LiDAR dominant beyond 5 m |
| **Object size policy** | Often allows larger max boxes; dynamic size filtering is usually off | Often uses tighter `max_object_size`; `target_constrain_size` can filter the dedicated dynamic output to expected object sizes |

In outdoor mode, the pipeline relies much more heavily on LiDAR clusters (Phase B and Case 2 of YOLO association) because the depth camera is effectively limited to ~4–5 m in most outdoor conditions. The classification pipeline must therefore depend more on Kalman kinematics and outside-FOV observed-motion checks than on dense depth point-cloud voting.

---

## 16. Tuning reference

This section groups the most important parameters by symptom. The parameter names are the YAML keys under `detector_node.ros__parameters`.

### YOLO association is too permissive

Use these when large boxes, trees, walls, or merged clusters become `yolo_candidate` because a small number of YOLO depth points falls inside them:

| Parameter | Effect |
|---|---|
| `yolo_point_fraction_threshold` | Higher value requires more of the YOLO foreground cloud to fall inside the 3D box |
| `yolo_sparse_large_box_volume_threshold` | Volume above which sparse-large protection activates |
| `yolo_sparse_large_min_points` | Minimum absolute YOLO point count required for large boxes |
| `yolo_sparse_large_min_density` | Minimum YOLO point density required for large boxes |
| `yolo_max_height_ratio` | Rejects boxes whose height is too large compared with YOLO foreground height |
| `yolo_min_height_span` | Clamp for YOLO foreground height to avoid near-zero ratios |
| `yolo_2d_iou_threshold` | Higher value makes LiDAR-only/no-depth fallback stricter |

Typical symptom: a person exits a car, but a nearby large tree/vehicle box becomes `potentially_dynamic`. First increase `yolo_sparse_large_min_points` or `yolo_sparse_large_min_density`; then tighten `yolo_max_height_ratio` if the false box is much taller than the person.

### YOLO association is too strict

Use these when valid people/cars fail to become `yolo_candidate`:

| Parameter | Effect |
|---|---|
| `yolo_point_fraction_threshold` | Lower value allows split/partial 3D boxes to associate |
| `yolo_depth_tolerance` | Indoor foreground depth window; increase if the object has depth spread |
| `yolo_outdoor_depth_tolerance` | Outdoor foreground depth window; increase for vehicles/cyclists at an angle |
| `yolo_sparse_large_box_volume_threshold` | Increase if normal objects are entering sparse-large mode |
| `yolo_sparse_large_min_points` | Lower if depth is sparse but associations are otherwise correct |
| `yolo_sparse_large_min_density` | Lower if valid large objects have sparse foreground points |

### Tracks swap IDs or attach to nearby structures

Relevant parameters:

| Parameter | Effect |
|---|---|
| `match_velocity_direction_score_weight` | Penalizes assignments inconsistent with motion direction |
| `max_velocity_direction_error_confirm` | Direction gate for tentative tracks |
| `max_velocity_direction_error_tracked` | Direction gate for confirmed tracks |
| `max_velocity_direction_error_confirm_dynamic` | Direction gate for dynamic tentative tracks |
| `max_velocity_direction_error_tracked_dynamic` | Direction gate for dynamic confirmed tracks |
| `max_relative_size_diff_match` | General size compatibility gate |
| `max_relative_size_diff_outside_fov` | Size compatibility gate when LiDAR-only outside the camera FOV |
| `yolo_base_mismatch_thresh` | Growth threshold for clearing sticky YOLO evidence outside FOV |
| `max_yolo_base_mismatch_frames` | How many consecutive growth frames before clearing YOLO evidence |

If the robot turns and a valid moving object changes apparent direction abruptly, prefer tuning `outside_fov_turn_*` parameters rather than disabling direction checks globally.

### Too many stationary objects are published

Relevant parameters:

| Parameter | Effect |
|---|---|
| `track_steady_objects` | If false, stationary non-YOLO confirmed tracks are suppressed from output |
| `stationary_speed_thresh` | KF speed below which a non-YOLO track is considered stationary |
| `min_confirm_hits` | Higher value delays confirmation |
| `min_confirm_hits_outside_fov` | Higher value delays confirmation for outside-FOV LiDAR-only tracks |

YOLO tracks are intentionally still published when stationary as `potentially_dynamic`.

### Dynamic classification is too sensitive

Relevant parameters:

| Parameter | Effect |
|---|---|
| `dynamic_velocity_threshold` | Main speed threshold for dynamic classification |
| `dynamic_voting_threshold` | Required ratio of moving point votes |
| `dynamic_consistency_threshold` | Consecutive evidence frames before dynamic confirmation |
| `dynamic_kf_vel_std_ratio` | Lower value makes KF-confidence shortcut stricter |
| `frames_force_dynamic` | Minimum dynamic frames needed to maintain label |
| `frames_force_dynamic_check_range` | Window used for dynamic continuity |
| `outside_fov_class_min_net_speed` | Minimum outside-FOV mean speed |
| `outside_fov_class_min_net_disp` | Minimum outside-FOV net displacement |
| `outside_fov_class_min_straightness` | Straightness gate for outside-FOV dynamic classification |

### Large static structures leak through detection

Relevant parameters:

| Parameter | Effect |
|---|---|
| `max_object_size` | Discards boxes larger than per-axis limits |
| `target_constrain_size` | Filters `/dynamic_bboxes` after classification, keeping only dynamic boxes close to configured target sizes |
| `target_object_size` | Expected object sizes used by that dynamic-output filter |
| `dbscan_refinement_enable` | Enables recursive split of large sparse clusters |
| `visual_dbscan_refinement_enable` | Same idea for depth-camera DBSCAN |
| `dbscan_refine_max_diagonal` | Large-cluster split trigger |
| `dbscan_refine_min_density` | Density threshold below which large clusters are split |

Outdoors, reducing `max_object_size` can stop oversized detections before tracking. Enabling `target_constrain_size` is a later dynamic-output filter: it can prevent wrongly shaped boxes from appearing on `/dynamic_bboxes`, but it does not resize the detection, and `trackedBBoxes_` / `ObstacleArray` can still carry the raw dynamic flag from `boxHist_`.

---

## 17. Debugging guide

This section maps common symptoms to the part of the pipeline most likely responsible.

### A large tree/wall/vehicle box becomes `potentially_dynamic`

Likely stage: YOLO association or sticky YOLO propagation.

Check:

1. `/filtered_before_yolo_bboxes`: if the large box already exists here, fusion/DBSCAN created it before YOLO.
2. `/yolo_points`: confirm whether the YOLO foreground points really fall inside the large box.
3. `yolo_sparse_large_*`: make the sparse-large gate stricter if only a few points are inside a large volume.
4. `yolo_max_height_ratio`: tighten if the large box is much taller than the actual YOLO foreground.
5. `max_yolo_base_mismatch_frames` and `yolo_base_mismatch_thresh`: if the issue happens outside FOV after a correct YOLO hit, the sticky YOLO flag may be surviving too long.

### A box label says `dynamic` but the marker color is not blue

Current marker code forces:

- `is_dynamic` → blue
- `is_potentially_dynamic` → green
- else caller color

If this happens again with the current code, first check whether the tracked marker and the dynamic marker are being observed from the same timestamp/frame. `mainCB()` refreshes the semantic flags in `trackedBBoxes_` after classification, so a persistent mismatch should no longer be explained by a one-cycle stale `trackedBBoxes_` snapshot. More likely causes are marker lifetime/timing in RViz, a different publisher/topic being displayed, or `target_constrain_size` filtering a box out of `/dynamic_bboxes` while `trackedBBoxes_` still carries the raw dynamic flag from `boxHist_`.

### A valid car/person beyond depth range does not become YOLO-associated

Likely stage: 2D IoU fallback.

Check:

1. Whether `filteredYoloPoints` is empty. If it is empty, the global projected-2D IoU fallback is used for all fused boxes. If it is not empty, each box still has a per-box IoU fallback, but only when that specific box has sparse depth support.
2. `yolo_2d_iou_threshold`: lower it if projected LiDAR boxes overlap the YOLO rectangle only partially.
3. Calibration: poor `camera_refined` extrinsic shifts projected 3D boxes and kills IoU.
4. LiDAR clustering: if the 3D box is too small/fragmented, the projection may not cover enough of the YOLO rectangle.

### A track ID swaps when objects pass near each other

Likely stage: association scoring/gates.

Check:

1. `match_velocity_direction_score_weight`.
2. `max_velocity_direction_error_*`.
3. `max_relative_size_diff_match`.
4. `confirmed_track_assoc_bonus` and `yolo_track_assoc_bonus`.
5. For outside-FOV turns, `outside_fov_turn_*` rather than the generic direction gates.

### Static clutter is tracked or published

Likely stage: confirmation/output suppression.

Check:

1. `track_steady_objects`: if true, non-YOLO stationary tracks are allowed into output.
2. `stationary_speed_thresh`: increase if very slow jitter should count as stationary.
3. `min_confirm_hits`: increase if one-off clutter becomes confirmed.
4. `max_object_size`: reduce if huge static clusters leak into the tracker.

### Dynamic object disappears too quickly

Likely stage: missed-frame lifetime or confirmation.

Check:

1. `max_missed_frames`.
2. `max_missed_frames_yolo`.
3. `min_confirm_hits` and `min_confirm_hits_outside_fov`.
4. Whether the object is being suppressed as stationary because KF speed dropped below `stationary_speed_thresh`.

### Dynamic object never becomes `dynamic`

Likely stage: classification.

Check:

1. `dynamic_velocity_threshold`: lower if the object moves slowly.
2. `dynamic_voting_threshold`: lower if point-cloud motion voting is too strict.
3. `dynamic_consistency_threshold`: lower if the object moves only briefly.
4. `dynamic_kf_vel_std_ratio`: increase if the KF speed is correct but noisy.
5. Outside FOV: `outside_fov_class_*` thresholds.
