/*
    FILE: wallDetector.h
    ---------------------------------
    Wall detection from LiDAR point clouds.
    Detects planar walls via RANSAC and publishes oriented bounding boxes
    as visualization_msgs::MarkerArray.
    Also estimates ground/roof height from the depth camera (RANSAC on
    bottom rows of the depth image) and publishes them for other nodes.
*/
#ifndef JO_DETECTOR_WALL_DETECTOR_H
#define JO_DETECTOR_WALL_DETECTOR_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cv_bridge/cv_bridge.hpp>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <random>
#include <numeric>
#include <unordered_map>
#include <functional>

namespace joDetector {

// =====================================================================
// PlaneModel
// =====================================================================
struct PlaneModel {
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    double d = 0.0;

    double distance(const Eigen::Vector3d& p) const {
        return std::abs(normal.dot(p) + d);
    }
};

// =====================================================================
// WallBBox – oriented bounding box for a wall segment
// =====================================================================
struct WallBBox {
    Eigen::Vector3d size     = Eigen::Vector3d::Zero();
    Eigen::Vector3d center   = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();

    const Eigen::Vector3d& get_size()     const { return size; }
    const Eigen::Vector3d& get_center()   const { return center; }
    const Eigen::Matrix3d& get_rotation() const { return rotation; }

    bool contains(const Eigen::Vector3d& p) const {
        Eigen::Vector3d local = rotation.transpose() * (p - center);
        Eigen::Vector3d half  = size * 0.5;
        return (std::abs(local.x()) <= half.x()) &&
               (std::abs(local.y()) <= half.y()) &&
               (std::abs(local.z()) <= half.z());
    }

    void transform(const Eigen::Isometry3d& T) {
        center   = T * center;
        rotation = T.rotation() * rotation;
    }
};

// =====================================================================
// WallBBoxRegistry – persistent wall tracking across frames
// =====================================================================
class WallBBoxRegistry {
public:
    using Ptr = std::shared_ptr<WallBBoxRegistry>;

    struct Config {
        double overlap_threshold    = 0.3;
        double merge_weight         = 0.3;
        bool   enable_expiry        = true;
        int    max_missed_frames    = 30;
        double min_normal_dot       = 0.9;
        double max_center_distance  = 5.0;
    };

    explicit WallBBoxRegistry(const Config& config);

    void update(const std::vector<WallBBox>& new_bboxes,
                const Eigen::Isometry3d& delta_pose);
    void remove_expired(const std::vector<bool>& empty_bboxes);
    const std::vector<WallBBox>& bboxes() const { return registry_; }

    // Merge nested/overlapping OBBs within a single detection frame before
    // passing them to update(). Uses IOV = vol_intersection / vol_smaller_OBB
    // so that containment (small box inside large box) is detected even when
    // IoU is low. Iterates until convergence.
    std::vector<WallBBox> mergeNested(const std::vector<WallBBox>& input,
                                      double iov_thresh) const;

private:
    struct OBB {
        Eigen::Vector3d center;
        Eigen::Matrix3d axes;
        Eigen::Vector3d half_extents;
    };

    Config config_;
    std::vector<WallBBox> registry_;
    std::vector<int> missed_frames_;

    OBB    to_obb(const WallBBox& bbox) const;
    double project_obb(const OBB& obb, const Eigen::Vector3d& axis) const;
    double intersection_volume(const OBB& a, const OBB& b) const;
    double iou(const WallBBox& a, const WallBBox& b) const;
    // IOV = vol_intersection / vol_smaller — detects containment of OBBs
    double iov(const WallBBox& a, const WallBBox& b) const;
    WallBBox merge(const WallBBox& existing, const WallBBox& incoming,
                   double weight) const;
    void transform_existing_bboxes(const Eigen::Isometry3d& delta_pose);
};

// =====================================================================
// WallDetector – ROS2 node logic
// =====================================================================
class WallDetector {
public:
    explicit WallDetector(rclcpp::Node::SharedPtr nh);

private:
    rclcpp::Node::SharedPtr nh_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ---- Subscribers ----
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    image_transport::Subscriber                                    depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr   camera_info_sub_;

    // ---- Publishers ----
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_markers_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ground_height_pub_;

    // Wall BBox Registry
    WallBBoxRegistry::Ptr bbox_registry_;

    // Pose tracking for delta computation
    Eigen::Isometry3d last_pose_;
    bool has_last_pose_ = false;

    // Calibration readiness: detection only starts after
    // the lidar→camera TF (from calibration ICP) is available
    bool calibration_ready_ = false;
    std::string calibration_child_frame_;  // e.g. "camera_refined"

    // ---- Wall detection enable flag ----
    bool enable_wall_detection_ = true;

    // ---- Ground / roof height estimation (depth-camera RANSAC) ----
    double ground_height_;
    double roof_height_;
    double ground_roof_offset_;
    double Conservativeoffset_;
    bool   ground_estimated_ = false;
    double ground_ema_alpha_;

    // Depth camera state
    cv::Mat depth_image_;                     // latest depth frame (CV_16UC1)
    Eigen::Matrix3d orientation_depth_;       // depth cam rotation in map frame
    Eigen::Vector3d position_depth_;          // depth cam position  in map frame
    bool depth_pose_valid_ = false;

    // Depth camera intrinsics & range
    double fx_, fy_, cx_, cy_;
    double depth_scale_;
    double depth_min_value_, depth_max_value_;
    int    skip_pixel_;
    int    ground_estim_bottom_fraction_;
    int    ground_estim_min_inliers_;

    // Frame names
    std::string depth_topic_;
    std::string depth_transport_;  // image_transport plugin depth_topic_ needs: "compressedDepth" (RealSense default), "zstd", "raw", ...
    std::string depth_frame_;   // TF frame for the depth camera
    std::string camera_info_topic_;  // live source for fx_/fy_/cx_/cy_ — see cameraInfoCallback()

    // ---- LiDAR wall-detection parameters ----
    std::string lidar_topic_;
    std::string odomFrame_;
    std::string lidar_frame_;
    bool use_tf_pose_;

    double voxel_resolution_;
    int    ransac_max_iterations_;
    double ransac_inlier_threshold_;
    int    ransac_min_inliers_;
    double ransac_confidence_;
    double wall_vertical_angle_deg_;
    int    max_planes_;

    double wall_bbox_max_aspect_ratio_;
    double nested_merge_iov_thresh_;   // IOV threshold for same-frame OBB merging

    // Optional horizontal range cap for wall search, applied AFTER voxel
    // filtering (on the already-downsampled centroids, so it's cheap
    // regardless of value) — see pointCloudCallback. 0 (default) = disabled,
    // unlimited range: a wall farther than any assumed "sensor range" is
    // still found, only bounded by point density at that distance.
    double wall_detection_max_range_;

    // RNG
    std::mt19937 rng_;

    // --- Callbacks ---
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);

    // --- Calibration gate ---
    void checkCalibrationReady();

    // --- Ground height estimation (depth-camera RANSAC, same as original) ---
    void estimateGroundHeight();

    // --- Pipeline helpers (wall detection) ---
    // cloud_frame_id: only used to stamp the debug filtered_cloud_pub_
    // publish — voxelFilter itself is frame-agnostic.
    std::vector<Eigen::Vector3d> voxelFilter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::string& cloud_frame_id);

    PlaneModel fitPlane(const Eigen::Vector3d& p0,
                        const Eigen::Vector3d& p1,
                        const Eigen::Vector3d& p2) const;

    std::vector<int> findInliers(const std::vector<Eigen::Vector3d>& pts,
                                 const PlaneModel& plane,
                                 double threshold) const;

    PlaneModel refitPlane(const std::vector<Eigen::Vector3d>& pts,
                          const std::vector<int>& inlier_idx) const;

    bool ransacOnce(std::vector<Eigen::Vector3d>& pts,
                    PlaneModel& best_plane);

    // up: world +Z expressed in whatever frame plane.normal is in — needed
    // because detection now runs in the lidar's own frame (see
    // pointCloudCallback), where "horizontal" isn't simply normal.z().
    bool isWallPlane(const PlaneModel& plane, const Eigen::Vector3d& up) const;

    std::vector<Eigen::Vector3d> extractLargestContiguousCluster(
        const std::vector<Eigen::Vector3d>& inliers,
        double cluster_eps) const;

    bool isWallBBoxCompact(const WallBBox& bbox) const;

    WallBBox buildWallBBox(const std::vector<Eigen::Vector3d>& inlier_pts,
                           const PlaneModel& plane) const;

    // --- Visualization ---
    void publishMarkers(const std_msgs::msg::Header& header);
};

} // namespace joDetector

#endif
