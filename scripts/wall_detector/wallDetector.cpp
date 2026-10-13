/*
    FILE: wallDetector.cpp
    ---------------------------------
    Wall detection from LiDAR scans using iterative RANSAC.
    Adapted from glim wall_filter / wall_bbox_registry to use
    PCL + Eigen (no glim / gtsam_points / spdlog dependencies).
*/
#include "wallDetector.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <algorithm>
#include <cmath>

namespace joDetector {

// =====================================================================
//  WallBBoxRegistry
// =====================================================================

WallBBoxRegistry::WallBBoxRegistry(const Config& config)
    : config_(config) {}

void WallBBoxRegistry::update(const std::vector<WallBBox>& new_bboxes,
                               const Eigen::Isometry3d& delta_pose) {
    transform_existing_bboxes(delta_pose);
    std::vector<bool> matched(registry_.size(), false);

    for (const auto& incoming : new_bboxes) {
        if (incoming.get_size().isZero()) continue;

        int    best_idx = -1;
        double best_iou = config_.overlap_threshold;

        for (int i = 0; i < static_cast<int>(registry_.size()); ++i) {
            const Eigen::Vector3d n_existing = registry_[i].get_rotation().col(0);
            const Eigen::Vector3d n_incoming = incoming.get_rotation().col(0);
            if (std::abs(n_existing.dot(n_incoming)) < config_.min_normal_dot)
                continue;

            if ((registry_[i].get_center() - incoming.get_center()).norm()
                    > config_.max_center_distance)
                continue;

            const double overlap = iou(registry_[i], incoming);
            if (overlap > best_iou) {
                best_iou = overlap;
                best_idx = i;
            }
        }

        if (best_idx >= 0) {
            registry_[best_idx]    = merge(registry_[best_idx], incoming,
                                           config_.merge_weight);
            matched[best_idx]      = true;
            missed_frames_[best_idx] = 0;
        } else {
            registry_.push_back(incoming);
            missed_frames_.push_back(0);
            matched.push_back(true);  // new entry: counts as matched on first frame
        }
    }

    if (config_.enable_expiry) {
        for (int i = 0; i < static_cast<int>(registry_.size()); ++i)
            if (!matched[i]) ++missed_frames_[i];

        for (int i = static_cast<int>(registry_.size()) - 1; i >= 0; --i) {
            if (missed_frames_[i] > config_.max_missed_frames) {
                registry_.erase(registry_.begin() + i);
                missed_frames_.erase(missed_frames_.begin() + i);
            }
        }
    }
}

void WallBBoxRegistry::remove_expired(const std::vector<bool>& empty_bboxes) {
    for (int i = static_cast<int>(registry_.size()) - 1; i >= 0; --i) {
        if (i < static_cast<int>(empty_bboxes.size()) && empty_bboxes[i]) {
            registry_.erase(registry_.begin() + i);
            missed_frames_.erase(missed_frames_.begin() + i);
        }
    }
}

void WallBBoxRegistry::transform_existing_bboxes(const Eigen::Isometry3d& delta_pose) {
    for (auto& bbox : registry_)
        bbox.transform(delta_pose);
}

// ---- OBB helpers ----

WallBBoxRegistry::OBB WallBBoxRegistry::to_obb(const WallBBox& bbox) const {
    OBB obb;
    obb.center       = bbox.get_center();
    obb.axes         = bbox.get_rotation();
    obb.half_extents = bbox.get_size() * 0.5;
    return obb;
}

double WallBBoxRegistry::project_obb(const OBB& obb,
                                      const Eigen::Vector3d& axis) const {
    return obb.half_extents[0] * std::abs(obb.axes.col(0).dot(axis))
         + obb.half_extents[1] * std::abs(obb.axes.col(1).dot(axis))
         + obb.half_extents[2] * std::abs(obb.axes.col(2).dot(axis));
}

double WallBBoxRegistry::intersection_volume(const OBB& a, const OBB& b) const {
    const Eigen::Vector3d t = b.center - a.center;

    double overlap[15];
    Eigen::Vector3d axes[15];

    int k = 0;
    for (int i = 0; i < 3; ++i) axes[k++] = a.axes.col(i);
    for (int i = 0; i < 3; ++i) axes[k++] = b.axes.col(i);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            axes[k] = a.axes.col(i).cross(b.axes.col(j));
            if (axes[k].norm() > 1e-6) axes[k].normalize();
            ++k;
        }

    for (int i = 0; i < 15; ++i) {
        if (axes[i].norm() < 1e-6) { overlap[i] = 1e9; continue; }
        const double center_proj = std::abs(t.dot(axes[i]));
        const double ra = project_obb(a, axes[i]);
        const double rb = project_obb(b, axes[i]);
        overlap[i] = std::max(0.0, ra + rb - center_proj);
        if (overlap[i] <= 0.0) return 0.0;
    }

    return std::min(overlap[0], 2.0 * a.half_extents[0])
         * std::min(overlap[1], 2.0 * a.half_extents[1])
         * std::min(overlap[2], 2.0 * a.half_extents[2]);
}

double WallBBoxRegistry::iou(const WallBBox& a, const WallBBox& b) const {
    const OBB oa = to_obb(a);
    const OBB ob = to_obb(b);

    const double vol_a = 8.0 * oa.half_extents.prod();
    const double vol_b = 8.0 * ob.half_extents.prod();
    if (vol_a < 1e-9 || vol_b < 1e-9) return 0.0;

    const double vol_inter = intersection_volume(oa, ob);
    return vol_inter / (vol_a + vol_b - vol_inter + 1e-9);
}

double WallBBoxRegistry::iov(const WallBBox& a, const WallBBox& b) const {
    const OBB oa = to_obb(a);
    const OBB ob = to_obb(b);

    const double vol_a = 8.0 * oa.half_extents.prod();
    const double vol_b = 8.0 * ob.half_extents.prod();
    if (vol_a < 1e-9 || vol_b < 1e-9) return 0.0;

    const double vol_inter = intersection_volume(oa, ob);
    return vol_inter / (std::min(vol_a, vol_b) + 1e-9);
}

std::vector<WallBBox> WallBBoxRegistry::mergeNested(
        const std::vector<WallBBox>& input, double iov_thresh) const {

    std::vector<WallBBox> current = input;

    // Iterative greedy merge: keep going until no pair exceeds the threshold.
    bool changed = true;
    while (changed) {
        changed = false;
        std::vector<bool> removed(current.size(), false);

        for (int i = 0; i < static_cast<int>(current.size()); ++i) {
            if (removed[i]) continue;
            for (int j = i + 1; j < static_cast<int>(current.size()); ++j) {
                if (removed[j]) continue;

                // Normals must be roughly parallel (same wall orientation)
                const Eigen::Vector3d ni = current[i].get_rotation().col(0);
                const Eigen::Vector3d nj = current[j].get_rotation().col(0);
                if (std::abs(ni.dot(nj)) < config_.min_normal_dot) continue;

                if (iov(current[i], current[j]) >= iov_thresh) {
                    // Merge j into i (i absorbs j), remove j
                    current[i] = merge(current[i], current[j], config_.merge_weight);
                    removed[j] = true;
                    changed    = true;
                }
            }
        }

        std::vector<WallBBox> next;
        next.reserve(current.size());
        for (int i = 0; i < static_cast<int>(current.size()); ++i)
            if (!removed[i]) next.push_back(current[i]);
        current = std::move(next);
    }

    return current;
}

WallBBox WallBBoxRegistry::merge(const WallBBox& existing,
                                  const WallBBox& incoming,
                                  double /*weight*/) const {
    const Eigen::Matrix3d& R    = existing.get_rotation();
    const Eigen::Vector3d& c_ex = existing.get_center();
    const Eigen::Vector3d  he_ex = existing.get_size() * 0.5;

    const Eigen::Vector3d c_in_local = R.transpose() * (incoming.get_center() - c_ex);
    const Eigen::Vector3d he_in      = incoming.get_size() * 0.5;
    const Eigen::Matrix3d R_in_local = R.transpose() * incoming.get_rotation();

    Eigen::Vector3d local_min = -he_ex;
    Eigen::Vector3d local_max =  he_ex;

    for (int sx : {-1, 1})
    for (int sy : {-1, 1})
    for (int sz : {-1, 1}) {
        const Eigen::Vector3d corner_local =
            R_in_local * Eigen::Vector3d(sx * he_in.x(),
                                          sy * he_in.y(),
                                          sz * he_in.z());
        const Eigen::Vector3d v = c_in_local + corner_local;
        local_min = local_min.cwiseMin(v);
        local_max = local_max.cwiseMax(v);
    }

    const Eigen::Vector3d new_size   = local_max - local_min;
    const Eigen::Vector3d new_center = c_ex + R * (0.5 * (local_max + local_min));

    WallBBox result;
    result.size     = new_size;
    result.center   = new_center;
    result.rotation = R;
    return result;
}

// =====================================================================
//  WallDetector
// =====================================================================

WallDetector::WallDetector(rclcpp::Node::SharedPtr nh)
    : nh_(nh), last_pose_(Eigen::Isometry3d::Identity()),
      calibration_ready_(false), ground_estimated_(false),
      orientation_depth_(Eigen::Matrix3d::Identity()),
      position_depth_(Eigen::Vector3d::Zero()),
      depth_pose_valid_(false),
      rng_(std::random_device{}())
{
    // ---- Load parameters ----
    auto p = [&](const std::string& name, auto def) {
        return nh_->get_parameter_or("wall_detector." + name, def);
    };

    lidar_topic_     = p("lidar_topic",     std::string("/velodyne_points"));
    odomFrame_       = p("odom_frame",      std::string("odom"));
    lidar_frame_     = p("lidar_frame",     std::string("velodyne"));
    use_tf_pose_     = p("use_tf_pose",     true);

    // Calibration frame: detection is gated until this TF child frame is available
    calibration_child_frame_ = p("calibration_child_frame", std::string("camera_refined"));

    // Wall detection enable/disable
    enable_wall_detection_ = p("enable_wall_detection", true);

    // Depth camera topic & frame
    depth_topic_ = p("depth_image_topic",
                     std::string("/front_camera/camera/depth/image_rect_raw"));
    // bags/*_validation_lab only records aligned_depth_to_color as zstd (no
    // compressedDepth variant at all — see calibration_icp_node's own
    // depth_transport param for the same situation); override both
    // together via cfg/launch when pointing this node at that bag.
    depth_transport_ = p("depth_transport", std::string("compressedDepth"));
    depth_frame_ = p("depth_frame", std::string("camera_refined"));

    // Depth intrinsics — starting values only, overwritten live the moment
    // a message arrives on camera_info_topic_ (see cameraInfoCallback() and
    // the subscription below); kept declared so the node still starts up
    // usably if that topic is misconfigured or never publishes.
    std::vector<double> intrinsics_default = {436.96, 436.96, 431.92, 240.13};
    std::vector<double> intrinsics = p("depth_intrinsics", intrinsics_default);
    fx_ = intrinsics[0];  fy_ = intrinsics[1];
    cx_ = intrinsics[2];  cy_ = intrinsics[3];
    camera_info_topic_ = p("camera_info_topic", std::string("/front_camera/camera/color/camera_info"));

    depth_scale_     = p("depth_scale_factor",  1000.0);
    depth_min_value_ = p("depth_min_value",     0.5);
    depth_max_value_ = p("depth_max_value",     4.5);
    skip_pixel_      = p("depth_skip_pixel",    2);

    // Ground estimation params
    ground_estim_bottom_fraction_ = p("ground_estim_bottom_fraction", 4);
    ground_estim_min_inliers_     = p("ground_estim_min_inliers",     50);
    ground_height_       = p("ground_height",        -0.7);
    ground_roof_offset_  = p("ground_roof_offset",  5.0);
    Conservativeoffset_  = p("ground_conservative_offset", 0.1);
    ground_ema_alpha_    = p("ground_ema_alpha",    0.05);
    roof_height_         = ground_height_ + ground_roof_offset_;

    // LiDAR wall filter params
    voxel_resolution_          = p("voxel_resolution",          0.25);
    ransac_max_iterations_     = p("ransac_max_iterations",     100);
    ransac_inlier_threshold_   = p("ransac_inlier_threshold",   0.07);
    ransac_min_inliers_        = p("ransac_min_inliers",        30);
    ransac_confidence_         = p("ransac_confidence",         0.99);
    wall_vertical_angle_deg_   = p("wall_vertical_angle_deg",  5.0);
    max_planes_                = p("max_planes",               8);
    wall_bbox_max_aspect_ratio_= p("wall_bbox_max_aspect_ratio", 5.0);
    nested_merge_iov_thresh_   = p("nested_merge_iov_thresh",   0.5);
    wall_detection_max_range_  = p("wall_detection_max_range",  0.0);  // 0 = unlimited

    // Registry config
    WallBBoxRegistry::Config reg_cfg;
    reg_cfg.overlap_threshold   = p("overlap_threshold",   0.3);
    reg_cfg.merge_weight        = p("merge_weight",        0.3);
    reg_cfg.enable_expiry       = p("enable_expiry",       true);
    reg_cfg.max_missed_frames   = p("max_missed_frames",  30);
    reg_cfg.min_normal_dot      = p("min_normal_dot",      0.9);
    reg_cfg.max_center_distance = p("max_center_distance", 5.0);

    bbox_registry_ = std::make_shared<WallBBoxRegistry>(reg_cfg);

    // ---- TF2 ----
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---- Publishers ----
    wall_markers_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/wall_detector/wall_markers", 10);
    filtered_cloud_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/wall_detector/filtered_cloud", 10);
    ground_height_pub_ = nh_->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/wall_detector/ground_height", 10);

    // ---- Subscribers ----
    // Depth image (for ground estimation — always active). Whatever
    // depth_transport_ names (default "compressedDepth", the RealSense
    // wrapper's default), the matching image_transport plugin decompresses
    // transparently on receipt, so depthImageCallback below always gets a
    // normal, uncompressed sensor_msgs::msg::Image. This is a single-topic
    // subscription (no cross-topic sync), so it's unaffected by the zstd
    // decode-stamp bug documented in calibration_icp_node.cpp — each frame
    // is processed independently as it arrives regardless of its stamp.
    depth_sub_ = image_transport::create_subscription(
        nh_.get(), depth_topic_,
        std::bind(&WallDetector::depthImageCallback, this, std::placeholders::_1),
        depth_transport_, rmw_qos_profile_sensor_data);

    // Live intrinsics — depth_intrinsics (already read into fx_/fy_/cx_/cy_
    // above) is only the bootstrap value until the first message here
    // arrives. See cameraInfoCallback()'s own comment.
    camera_info_sub_ = nh_->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, rclcpp::SensorDataQoS(),
        std::bind(&WallDetector::cameraInfoCallback, this, std::placeholders::_1));

    // LiDAR (for wall detection — gated on enable flag and calibration)
    lidar_sub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, rclcpp::SensorDataQoS(),
        std::bind(&WallDetector::pointCloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(nh_->get_logger(),
        "[WallDetector] Depth on '%s' (transport=%s), LiDAR on '%s', walls %s, "
        "gated on calibration frame '%s', intrinsics bootstrap fx=%.3f fy=%.3f cx=%.3f cy=%.3f live from '%s'",
        depth_topic_.c_str(), depth_transport_.c_str(), lidar_topic_.c_str(),
        enable_wall_detection_ ? "ENABLED" : "DISABLED",
        calibration_child_frame_.c_str(),
        fx_, fy_, cx_, cy_, camera_info_topic_.c_str());
}

// =====================================================================
// Main callback
// =====================================================================

void WallDetector::pointCloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    // 0. Gate on calibration readiness AND wall detection flag
    if (!calibration_ready_) {
        checkCalibrationReady();
        if (!calibration_ready_) return;
    }
    if (!enable_wall_detection_) return;

    // 1. Convert to PCL. Deliberately stays in the LiDAR's own frame (e.g.
    // "velodyne") for the whole detection pipeline below: RANSAC, plane
    // classification, clustering and bbox-building are all relative
    // geometry (distances, dot products, local PCA) — they give identical
    // results no matter which frame the points are expressed in. Only the
    // small set of FOUND WallBBoxes (center + rotation + size, ~15 numbers
    // each — see WallBBox::transform()) needs transforming into the world
    // frame afterward, at step 6c below — vastly cheaper every scan than
    // transforming the ~29k raw points (previously done here) or even the
    // few-hundred voxelized centroids.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) return;

    // 2. Look up lidar -> map/odom once. Needed for two things below:
    // rotating the ground/roof planes and the "up" direction into the lidar
    // frame (so height/wall classification stays correct even if the
    // sensor isn't perfectly level), and transforming the final WallBBoxes
    // for the persistent registry (which only makes sense in one fixed
    // world frame). Detection can't usefully continue without it — unlike
    // the old code, which silently fell back to "sensor frame" processing
    // on a failed lookup and could mix frames across scans in the registry
    // — so a failed/disabled lookup skips this scan outright.
    if (!use_tf_pose_) {
        RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 5000,
            "[WallDetector] use_tf_pose is false; wall detection needs it to place boxes in "
            "the world-frame registry, skipping.");
        return;
    }
    Eigen::Isometry3d T_map_lidar = Eigen::Isometry3d::Identity();
    try {
        auto tf = tf_buffer_->lookupTransform(
            odomFrame_, msg->header.frame_id,
            tf2::TimePointZero, tf2::durationFromSec(0.1));

        T_map_lidar.translation() = Eigen::Vector3d(
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z);
        Eigen::Quaterniond q(
            tf.transform.rotation.w,
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z);
        T_map_lidar.linear() = q.toRotationMatrix();
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 2000,
            "[WallDetector] TF lookup failed: %s — skipping this scan", ex.what());
        return;
    }

    // World +Z expressed in the lidar frame — reused below for both the
    // ground/roof crop and isWallPlane()'s horizontal-normal test.
    const Eigen::Vector3d up_lidar = T_map_lidar.linear().transpose() * Eigen::Vector3d::UnitZ();

    // 3. Ground/roof height crop, in lidar frame, BEFORE voxel filtering.
    // ground_height_/roof_height_ are map-frame Z values; a point p_lidar's
    // map-frame height is up_lidar·p_lidar + T_map_lidar.translation().z()
    // (by up_lidar's own definition above), so the crop reduces to bounding
    // that one dot product — no per-point transform needed. This is a
    // vertical crop only — deliberately NOT a horizontal/range crop, so a
    // wall farther away than any assumed "sensor range" is still found;
    // detecting a wall further out is instead a question of point density
    // (VLP-16 angular resolution thins out with distance) — tune
    // ransac_min_inliers_/voxel_resolution_ down if a real far wall in your
    // scene is being missed for lack of inliers, not this crop.
    if (ground_estimated_) {
        const double tz = T_map_lidar.translation().z();
        const double lo = ground_height_ - tz;
        const double hi = roof_height_   - tz;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZ>);
        cropped->points.reserve(cloud->points.size());
        for (const auto& pt : cloud->points) {
            const double h = up_lidar.x() * pt.x + up_lidar.y() * pt.y + up_lidar.z() * pt.z;
            if (h >= lo && h <= hi) cropped->points.push_back(pt);
        }
        cropped->width    = static_cast<uint32_t>(cropped->points.size());
        cropped->height   = 1;
        cropped->is_dense = cloud->is_dense;
        cloud = cropped;
    }

    // 4. Voxel filter → extract centroids (still lidar frame)
    std::vector<Eigen::Vector3d> centroids = voxelFilter(cloud, msg->header.frame_id);
    if (static_cast<int>(centroids.size()) < 3) return;

    // 4b. Optional horizontal range cap, AFTER voxel filtering — i.e. on
    // the few hundred already-downsampled centroids, not the raw ~29k-point
    // scan, so enabling it costs next to nothing regardless of the range
    // chosen. Disabled (0) by default: unlimited range, same reasoning as
    // step 3 above. In lidar frame the sensor sits at the origin, so this
    // is a plain distance-from-(0,0) check.
    if (wall_detection_max_range_ > 0.0) {
        const double max_range2 = wall_detection_max_range_ * wall_detection_max_range_;
        std::vector<Eigen::Vector3d> ranged;
        ranged.reserve(centroids.size());
        for (const auto& c : centroids) {
            if (c.x() * c.x() + c.y() * c.y() <= max_range2) ranged.push_back(c);
        }
        centroids = std::move(ranged);
        if (static_cast<int>(centroids.size()) < 3) return;
    }

    // 5. Iterative RANSAC — wall planes only (lidar frame; frame-agnostic
    // geometry, see isWallPlane's up_lidar argument)
    std::vector<PlaneModel> wall_planes;
    std::vector<Eigen::Vector3d> remaining = centroids;

    for (int p = 0; p < max_planes_; ++p) {
        if (static_cast<int>(remaining.size()) < ransac_min_inliers_) break;
        PlaneModel plane;
        if (!ransacOnce(remaining, plane)) break;

        if (isWallPlane(plane, up_lidar))
            wall_planes.push_back(plane);
    }

    // 6. Build wall bboxes from wall planes (lidar frame), then transform
    // each FOUND box — not the point cloud — into the world frame.
    std::vector<WallBBox> wall_bboxes;
    wall_bboxes.reserve(wall_planes.size());

    for (const auto& plane : wall_planes) {
        std::vector<Eigen::Vector3d> inliers;
        for (const auto& c : centroids) {
            if (plane.distance(c) < ransac_inlier_threshold_)
                inliers.push_back(c);
        }
        if (static_cast<int>(inliers.size()) < ransac_min_inliers_) continue;

        const double cluster_eps = voxel_resolution_ * 2.5;
        auto contiguous = extractLargestContiguousCluster(inliers, cluster_eps);

        if (static_cast<int>(contiguous.size()) < ransac_min_inliers_) continue;

        WallBBox bbox = buildWallBBox(contiguous, plane);
        if (!isWallBBoxCompact(bbox)) continue;

        // 6c. Only the found box (center/rotation) gets transformed — see
        // step 1's comment. Size is frame-invariant under a rigid transform.
        bbox.transform(T_map_lidar);
        wall_bboxes.push_back(bbox);
    }

    // 7. Merge nested/overlapping OBBs within this frame, then update
    // registry — both already operate purely on WallBBox center/rotation/
    // size, so nothing here needed to change for the lidar-frame detection
    // above; every box entering this step is already in odomFrame_.
    if (!wall_bboxes.empty()) {
        const std::vector<WallBBox> merged =
            bbox_registry_->mergeNested(wall_bboxes, nested_merge_iov_thresh_);
        Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
        bbox_registry_->update(merged, delta);
    }

    // 8. Publish markers — always odomFrame_ now: the registry only ever
    // holds boxes already transformed there (step 6c).
    std_msgs::msg::Header header;
    header.stamp    = msg->header.stamp;
    header.frame_id = odomFrame_;
    publishMarkers(header);

    // RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 2000,
    //     "[WallDetector] %zu centroids, %zu wall planes, %zu registry bboxes, "
    //     "ground=%.2f roof=%.2f",
    //     centroids.size(), wall_planes.size(),
    //     bbox_registry_->bboxes().size(),
    //     ground_height_, roof_height_);
}

// =====================================================================
// Voxel filter: downsample then extract centroids
// =====================================================================

std::vector<Eigen::Vector3d> WallDetector::voxelFilter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::string& cloud_frame_id) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(static_cast<float>(voxel_resolution_),
                   static_cast<float>(voxel_resolution_),
                   static_cast<float>(voxel_resolution_));
    vg.filter(*filtered);

    // Publish filtered cloud for debugging. cloud is in the lidar's own
    // frame now (detection no longer transforms the cloud — see
    // pointCloudCallback), so just pass its frame straight through instead
    // of guessing odomFrame_/"velodyne"; rviz (or anything else) transforms
    // it to whatever fixed frame it's displayed in via TF as usual.
    if (filtered_cloud_pub_->get_subscription_count() > 0) {
        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(*filtered, out_msg);
        out_msg.header.frame_id = cloud_frame_id;
        out_msg.header.stamp = nh_->now();
        filtered_cloud_pub_->publish(out_msg);
    }

    std::vector<Eigen::Vector3d> centroids;
    centroids.reserve(filtered->size());
    for (const auto& pt : filtered->points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
            centroids.emplace_back(pt.x, pt.y, pt.z);
    }
    return centroids;
}

// =====================================================================
// RANSAC helpers
// =====================================================================

PlaneModel WallDetector::fitPlane(const Eigen::Vector3d& p0,
                                   const Eigen::Vector3d& p1,
                                   const Eigen::Vector3d& p2) const {
    Eigen::Vector3d n    = (p1 - p0).cross(p2 - p0);
    const double    norm = n.norm();

    PlaneModel plane;
    if (norm < 1e-9) {
        plane.normal = Eigen::Vector3d::UnitZ();
        plane.d      = -plane.normal.dot(p0);
        return plane;
    }
    plane.normal = n / norm;
    plane.d      = -plane.normal.dot(p0);
    return plane;
}

std::vector<int> WallDetector::findInliers(
        const std::vector<Eigen::Vector3d>& pts,
        const PlaneModel& plane,
        double threshold) const {
    std::vector<int> inliers;
    inliers.reserve(pts.size() / 4);
    for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
        if (std::abs(plane.normal.dot(pts[i]) + plane.d) < threshold)
            inliers.push_back(i);
    }
    return inliers;
}

PlaneModel WallDetector::refitPlane(const std::vector<Eigen::Vector3d>& pts,
                                     const std::vector<int>& inlier_idx) const {
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (int idx : inlier_idx) mean += pts[idx];
    mean /= static_cast<double>(inlier_idx.size());

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (int idx : inlier_idx) {
        Eigen::Vector3d d = pts[idx] - mean;
        cov += d * d.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();

    PlaneModel plane;
    plane.normal = normal;
    plane.d      = -normal.dot(mean);
    return plane;
}

bool WallDetector::ransacOnce(std::vector<Eigen::Vector3d>& pts,
                               PlaneModel& best_plane) {
    const int n = static_cast<int>(pts.size());
    if (n < 3) return false;

    std::uniform_int_distribution<int> dist(0, n - 1);

    int              best_count = 0;
    std::vector<int> best_inliers;

    for (int iter = 0; iter < ransac_max_iterations_; ++iter) {
        int i0, i1, i2;
        i0 = dist(rng_);
        do { i1 = dist(rng_); } while (i1 == i0);
        do { i2 = dist(rng_); } while (i2 == i0 || i2 == i1);

        PlaneModel candidate = fitPlane(pts[i0], pts[i1], pts[i2]);
        if (candidate.normal.norm() < 0.5) continue;

        std::vector<int> inliers =
            findInliers(pts, candidate, ransac_inlier_threshold_);

        if (static_cast<int>(inliers.size()) > best_count) {
            best_count   = static_cast<int>(inliers.size());
            best_inliers = std::move(inliers);
            best_plane   = candidate;

            const double w  = static_cast<double>(best_count) / n;
            const double w3 = w * w * w;
            if (w3 > 1.0 - 1e-9) break;
            const double n_iter = std::log(1.0 - ransac_confidence_)
                                / std::log(1.0 - w3 + 1e-10);
            if (iter + 1 >= static_cast<int>(std::ceil(n_iter))) break;
        }
    }

    if (best_count < ransac_min_inliers_) return false;

    best_plane = refitPlane(pts, best_inliers);

    // Remove inliers for next RANSAC iteration
    std::vector<bool> is_inlier(n, false);
    for (int idx : best_inliers) is_inlier[idx] = true;

    std::vector<Eigen::Vector3d> rest;
    rest.reserve(n - best_count);
    for (int i = 0; i < n; ++i)
        if (!is_inlier[i]) rest.push_back(pts[i]);

    pts = std::move(rest);
    return true;
}

// =====================================================================
// Plane classification
// =====================================================================

bool WallDetector::isWallPlane(const PlaneModel& plane, const Eigen::Vector3d& up) const {
    const double sin_thr = std::sin(wall_vertical_angle_deg_ * M_PI / 180.0);
    // A wall's normal is horizontal, i.e. (near-)perpendicular to "up" — was
    // plane.normal.z() when detection ran in map frame (where up = UnitZ());
    // now generalized to whatever frame plane.normal is actually in.
    return std::abs(plane.normal.dot(up)) < sin_thr;
}

// =====================================================================
// Contiguous cluster extraction (union-find)
// =====================================================================

std::vector<Eigen::Vector3d> WallDetector::extractLargestContiguousCluster(
        const std::vector<Eigen::Vector3d>& inliers,
        double cluster_eps) const {

    const int n = static_cast<int>(inliers.size());
    if (n == 0) return {};

    // Union-Find
    std::vector<int> parent(n);
    std::iota(parent.begin(), parent.end(), 0);

    std::function<int(int)> find = [&](int x) -> int {
        return parent[x] == x ? x : parent[x] = find(parent[x]);
    };
    auto unite = [&](int a, int b) {
        parent[find(a)] = find(b);
    };

    if (n <= 500) {
        const double eps2 = cluster_eps * cluster_eps;
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j)
                if ((inliers[i] - inliers[j]).squaredNorm() < eps2)
                    unite(i, j);
    } else {
        // Use PCL KdTree for large datasets
        pcl::PointCloud<pcl::PointXYZ>::Ptr kd_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        kd_cloud->resize(n);
        for (int i = 0; i < n; ++i) {
            (*kd_cloud)[i].x = static_cast<float>(inliers[i].x());
            (*kd_cloud)[i].y = static_cast<float>(inliers[i].y());
            (*kd_cloud)[i].z = static_cast<float>(inliers[i].z());
        }

        pcl::KdTreeFLANN<pcl::PointXYZ> tree;
        tree.setInputCloud(kd_cloud);

        const int K = std::min(n, 16);
        std::vector<int> idx(K);
        std::vector<float> sq(K);

        for (int i = 0; i < n; ++i) {
            int found = tree.nearestKSearch((*kd_cloud)[i], K, idx, sq);
            for (int k = 0; k < found; ++k)
                if (sq[k] < static_cast<float>(cluster_eps * cluster_eps))
                    unite(i, idx[k]);
        }
    }

    // Find largest component
    std::unordered_map<int, std::vector<int>> components;
    for (int i = 0; i < n; ++i)
        components[find(i)].push_back(i);

    int best_root = -1;
    int best_size = 0;
    for (const auto& [root, members] : components) {
        if (static_cast<int>(members.size()) > best_size) {
            best_size = static_cast<int>(members.size());
            best_root = root;
        }
    }

    if (best_root < 0) return inliers;

    std::vector<Eigen::Vector3d> result;
    result.reserve(best_size);
    for (int idx : components[best_root])
        result.push_back(inliers[idx]);

    return result;
}

// =====================================================================
// BBox compactness check
// =====================================================================

bool WallDetector::isWallBBoxCompact(const WallBBox& bbox) const {
    const Eigen::Vector3d& s = bbox.get_size();
    if (s.minCoeff() < 1e-3) return false;

    Eigen::Vector3d dims = s;
    std::sort(dims.data(), dims.data() + 3);
    const double aspect = dims[2] / dims[1];
    return aspect < wall_bbox_max_aspect_ratio_;
}

// =====================================================================
// Build OBB from plane inliers
// =====================================================================

WallBBox WallDetector::buildWallBBox(
        const std::vector<Eigen::Vector3d>& inlier_pts,
        const PlaneModel& plane) const {

    if (inlier_pts.empty()) {
        return WallBBox{};
    }

    const Eigen::Vector3d n = plane.normal.normalized();

    Eigen::Vector3d ref = Eigen::Vector3d::UnitX();
    if (std::abs(n.dot(ref)) > 0.9)
        ref = Eigen::Vector3d::UnitY();

    const Eigen::Vector3d right = (ref - n.dot(ref) * n).normalized();
    const Eigen::Vector3d up    = n.cross(right).normalized();

    Eigen::Matrix3d R;
    R.col(0) = n;
    R.col(1) = right;
    R.col(2) = up;

    const Eigen::Vector3d origin = inlier_pts[0];

    Eigen::Vector3d local_min( 1e9,  1e9,  1e9);
    Eigen::Vector3d local_max(-1e9, -1e9, -1e9);

    for (const auto& p : inlier_pts) {
        const Eigen::Vector3d local = R.transpose() * (p - origin);
        local_min = local_min.cwiseMin(local);
        local_max = local_max.cwiseMax(local);
    }

    Eigen::Vector3d size = local_max - local_min;
    // Minimum thickness along normal (axis 0)
    size.x() = std::max(size.x(), ransac_inlier_threshold_ * 2.0);

    const Eigen::Vector3d local_center = 0.5 * (local_max + local_min);
    const Eigen::Vector3d world_center = R * local_center + origin;

    WallBBox result;
    result.size     = size;
    result.center   = world_center;
    result.rotation = R;
    return result;
}

// =====================================================================
// Publish MarkerArray for walls
// =====================================================================

void WallDetector::publishMarkers(const std_msgs::msg::Header& header) {

    const auto& bboxes = bbox_registry_->bboxes();

    // --- Wall markers ---
    {
        visualization_msgs::msg::MarkerArray markers;

        // Delete-all marker to clear stale markers
        visualization_msgs::msg::Marker del;
        del.header = header;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(del);

        for (size_t i = 0; i < bboxes.size(); ++i) {
            const auto& bbox = bboxes[i];

            visualization_msgs::msg::Marker m;
            m.header    = header;
            m.ns        = "wall_bbox";
            m.id        = static_cast<int>(i);
            m.type      = visualization_msgs::msg::Marker::CUBE;
            m.action    = visualization_msgs::msg::Marker::ADD;
            m.lifetime  = rclcpp::Duration::from_seconds(0.5);

            // Position
            m.pose.position.x = bbox.center.x();
            m.pose.position.y = bbox.center.y();
            m.pose.position.z = bbox.center.z();

            // Orientation from rotation matrix
            Eigen::Quaterniond q(bbox.rotation);
            m.pose.orientation.x = q.x();
            m.pose.orientation.y = q.y();
            m.pose.orientation.z = q.z();
            m.pose.orientation.w = q.w();

            // Scale = size
            m.scale.x = bbox.size.x();
            m.scale.y = bbox.size.y();
            m.scale.z = bbox.size.z();

            // Color: semi-transparent cyan for walls
            m.color.r = 0.0f;
            m.color.g = 0.8f;
            m.color.b = 0.8f;
            m.color.a = 0.35f;

            markers.markers.push_back(m);

            // Also add wireframe (LINE_LIST) for better visibility
            visualization_msgs::msg::Marker wire;
            wire.header    = header;
            wire.ns        = "wall_wireframe";
            wire.id        = static_cast<int>(i);
            wire.type      = visualization_msgs::msg::Marker::LINE_LIST;
            wire.action    = visualization_msgs::msg::Marker::ADD;
            wire.lifetime  = rclcpp::Duration::from_seconds(0.5);
            wire.scale.x   = 0.03;
            wire.color.r   = 0.0f;
            wire.color.g   = 1.0f;
            wire.color.b   = 1.0f;
            wire.color.a   = 1.0f;
            wire.pose.position.x = bbox.center.x();
            wire.pose.position.y = bbox.center.y();
            wire.pose.position.z = bbox.center.z();
            wire.pose.orientation = m.pose.orientation;

            double hx = bbox.size.x() * 0.5;
            double hy = bbox.size.y() * 0.5;
            double hz = bbox.size.z() * 0.5;

            geometry_msgs::msg::Point c[8];
            c[0].x = -hx; c[0].y = -hy; c[0].z = -hz;
            c[1].x = -hx; c[1].y =  hy; c[1].z = -hz;
            c[2].x =  hx; c[2].y =  hy; c[2].z = -hz;
            c[3].x =  hx; c[3].y = -hy; c[3].z = -hz;
            c[4].x = -hx; c[4].y = -hy; c[4].z =  hz;
            c[5].x = -hx; c[5].y =  hy; c[5].z =  hz;
            c[6].x =  hx; c[6].y =  hy; c[6].z =  hz;
            c[7].x =  hx; c[7].y = -hy; c[7].z =  hz;

            int edges[12][2] = {
                {0,1},{1,2},{2,3},{3,0},
                {4,5},{5,6},{6,7},{7,4},
                {0,4},{1,5},{2,6},{3,7}
            };
            for (int e = 0; e < 12; ++e) {
                wire.points.push_back(c[edges[e][0]]);
                wire.points.push_back(c[edges[e][1]]);
            }

            markers.markers.push_back(wire);
        }

        wall_markers_pub_->publish(markers);
    }
}

// =====================================================================
// Check whether the camera-lidar calibration TF is available
// =====================================================================
void WallDetector::checkCalibrationReady() {
    try {
        tf_buffer_->lookupTransform(
            lidar_frame_, calibration_child_frame_,
            tf2::TimePointZero, tf2::durationFromSec(0.0));
        calibration_ready_ = true;
        // RCLCPP_INFO(nh_->get_logger(),
        //     "[WallDetector] Calibration TF '%s' -> '%s' available – detection enabled",
        //     lidar_frame_.c_str(), calibration_child_frame_.c_str());
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 3000,
            "[WallDetector] Waiting for calibration TF '%s' -> '%s': %s",
            lidar_frame_.c_str(), calibration_child_frame_.c_str(), ex.what());
    }
}

// =====================================================================
// Depth image callback — updates camera pose & runs ground estimation
// =====================================================================
// fx_/fy_/cx_/cy_ start out at whatever depth_intrinsics declared (see that
// param's own comment above) and are overwritten here on every message —
// camera_info_topic_ should be the COLOR camera's own info topic (K is in
// pixel units against that camera's resolution, matching depth_topic_ being
// an aligned-to-color stream). Not gated to "first message only": intrinsics
// are physically fixed for a given camera, so re-applying the same K on
// every message is a harmless no-op.
void WallDetector::cameraInfoCallback(
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];
}

void WallDetector::depthImageCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& msg) {

    // Gate on calibration readiness
    if (!calibration_ready_) {
        checkCalibrationReady();
        if (!calibration_ready_) {
            RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 3000,
                "[WallDetector] depthImageCB: calibration NOT ready, skipping");
            return;
        }
    }

    // Convert depth image
    cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(msg, msg->encoding);
    if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        imgPtr->image.convertTo(imgPtr->image, CV_16UC1, depth_scale_);
    }
    imgPtr->image.copyTo(depth_image_);

    // RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 3000,
    //     "[WallDetector] depthImageCB: got depth %dx%d, encoding='%s'",
    //     depth_image_.cols, depth_image_.rows, msg->encoding.c_str());

    // Get camera pose via TF (in odom frame if use_tf_pose_, otherwise in lidar/sensor frame)
    const std::string& depth_target_frame = use_tf_pose_ ? odomFrame_ : lidar_frame_;
    try {
        auto tf = tf_buffer_->lookupTransform(
            depth_target_frame, depth_frame_,
            tf2::TimePointZero, tf2::durationFromSec(0.1));

        Eigen::Quaterniond q(
            tf.transform.rotation.w,
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z);
        orientation_depth_ = q.toRotationMatrix();
        position_depth_ = Eigen::Vector3d(
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z);
        depth_pose_valid_ = true;
        // RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 3000,
        //     "[WallDetector] depthImageCB: TF ok, cam pos=(%.2f,%.2f,%.2f)",
        //     position_depth_.x(), position_depth_.y(), position_depth_.z());
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 2000,
            "[WallDetector] Depth TF lookup failed: %s", ex.what());
        return;
    }

    // Run ground estimation (always, regardless of wall detection flag)
    estimateGroundHeight();
}

// =====================================================================
// Estimate ground height from depth image (exact replica of original)
// =====================================================================
void WallDetector::estimateGroundHeight() {
    if (depth_image_.empty()) {
        RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 3000,
            "[WallDetector] estimateGroundHeight: depth_image_ empty");
        return;
    }
    if (!depth_pose_valid_) {
        RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 3000,
            "[WallDetector] estimateGroundHeight: depth_pose not valid");
        return;
    }

    // Sanity check: if the depth camera's y-axis in the map frame
    // is far from pointing downward, the bottom rows don't reliably see ground.
    Eigen::Vector3d camYinMap = orientation_depth_.col(1);
    if (std::abs(camYinMap(2)) < 0.5) {
        RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 3000,
            "[WallDetector] estimateGroundHeight: camY.z=%.3f (abs < 0.5), "
            "camera not pointing down enough. camY=(%.3f,%.3f,%.3f)",
            camYinMap(2), camYinMap(0), camYinMap(1), camYinMap(2));
        return;
    }

    int rows = depth_image_.rows;
    int cols = depth_image_.cols;
    int startRow = rows - rows / ground_estim_bottom_fraction_;

    const double inv_factor = 1.0 / depth_scale_;
    const double inv_fx     = 1.0 / fx_;
    const double inv_fy     = 1.0 / fy_;

    std::vector<double> zValues;
    zValues.reserve(
        (rows - startRow) * cols /
        (skip_pixel_ * skip_pixel_));

    for (int v = startRow; v < rows; v += skip_pixel_) {
        const uint16_t* rowPtr = depth_image_.ptr<uint16_t>(v);
        for (int u = 0; u < cols; u += skip_pixel_) {
            double depth = rowPtr[u] * inv_factor;
            if (depth < depth_min_value_ || depth > depth_max_value_) {
                continue;
            }
            Eigen::Vector3d ptCam;
            ptCam(0) = (u - cx_) * depth * inv_fx;
            ptCam(1) = (v - cy_) * depth * inv_fy;
            ptCam(2) = depth;

            // project into map frame — z is height above map origin
            Eigen::Vector3d ptMap =
                orientation_depth_ * ptCam + position_depth_;

            // keep only points that are plausibly on the floor:
            // below the camera and within a reasonable absolute height range
            if (ptMap(2) > position_depth_(2)) {
                continue;
            }

            zValues.push_back(ptMap(2));
        }
    }

    if (static_cast<int>(zValues.size()) < ground_estim_min_inliers_) {
        RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 3000,
            "[WallDetector] estimateGroundHeight: only %zu z-values (need %d)",
            zValues.size(), ground_estim_min_inliers_);
        return;
    }

    // RANSAC on z = c (horizontal plane in map frame)
    const double inlierThresh = 0.05;   // 5 cm
    const int    maxIter      = 60;

    int    bestInliers = 0;
    double bestZ       = 0.0;

    std::uniform_int_distribution<int> dist(
        0, static_cast<int>(zValues.size()) - 1);

    for (int iter = 0; iter < maxIter; ++iter) {
        double zHyp = zValues[dist(rng_)];

        int    inliers = 0;
        double zSum    = 0.0;
        for (double z : zValues) {
            if (std::abs(z - zHyp) < inlierThresh) {
                ++inliers;
                zSum += z;
            }
        }

        if (inliers > bestInliers) {
            bestInliers = inliers;
            bestZ       = zSum / inliers;
        }
    }

    if (bestInliers < ground_estim_min_inliers_) {
        RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 3000,
            "[WallDetector] estimateGroundHeight: RANSAC bestInliers=%d < %d",
            bestInliers, ground_estim_min_inliers_);
        return;
    }

    if (!ground_estimated_) {
        ground_height_    = bestZ + Conservativeoffset_;
        ground_estimated_ = true;
    } else {
        const double alpha = ground_ema_alpha_;
        ground_height_ = (1.0 - alpha) * ground_height_ + alpha * (bestZ + Conservativeoffset_);
    }

    roof_height_ = ground_height_ + ground_roof_offset_;

    // Publish ground/roof height
    std_msgs::msg::Float64MultiArray gh_msg;
    gh_msg.data.resize(2);
    gh_msg.data[0] = ground_height_;
    gh_msg.data[1] = roof_height_;
    ground_height_pub_->publish(gh_msg);

    // RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 2000,
    //     "[WallDetector] Ground=%.3f  Roof=%.3f  (RANSAC inliers=%d, zValues=%zu)",
    //     ground_height_, roof_height_, bestInliers, zValues.size());
}

} // namespace joDetector
