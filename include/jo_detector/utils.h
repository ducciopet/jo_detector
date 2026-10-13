 /*
 	FILE: utils.h
 	--------------------------
 	function utils for detectors
 */

#ifndef JO_DETECTOR_UTILS_H
#define JO_DETECTOR_UTILS_H
#include <iomanip>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <Eigen/Eigen>
#include <algorithm>
#include <limits>

namespace joDetector{
    const double PI_const = 3.1415926;

    struct Node
    {
        int idx;
        bool is_group1; // true = UV, false = DB
    };

    struct Edge
    {
        Node parent;
        Node child;
        double weight;
    };

    struct matchCandidate
    {
        int currIdx;
        int prevIdx;
        double score;
    };

    struct clusterGeometry
    {
        double numPoints;
        double sizeX;
        double sizeY;
        double sizeZ;
        double varX;
        double varY;
        double varZ;
    };

    struct box3D
    {
        /* data */
        double x, y, z;
        double x_width, y_width, z_width;
        double id;
        double Vx=0, Vy=0, Vz=0;
        double Ax=0, Ay=0, Az=0;
        bool is_yolo_candidate=false; // true: detected by yolo as a target dynamic class
        bool is_dynamic=false; // false: not detected as dynamic(either yolo or classificationCB), true: detected as dynamic
        bool fix_size=false; // flag to force future boxes to fix size
        bool is_dynamic_candidate=false;
        bool is_potentially_dynamic=false; // true: YOLO-identified but KF speed ~ 0 (stationary recognized object)
        bool is_estimated=false;
        uint32_t track_age=0; // frames this track has been alive
    };

    inline geometry_msgs::msg::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
    {
        if (yaw > PI_const){
            yaw = yaw - 2*PI_const;
        }
        tf2::Quaternion quaternion_tf2;
        quaternion_tf2.setRPY(roll, pitch, yaw);
        geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
        return quaternion;
    }

    inline double rpy_from_quaternion(const geometry_msgs::msg::Quaternion& quat){
        // return is [0, 2pi]
        tf2::Quaternion tf_quat;
        tf2::convert(quat, tf_quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
        return yaw;
    }

    inline void rpy_from_quaternion(const geometry_msgs::msg::Quaternion& quat, double &roll, double &pitch, double &yaw){
        tf2::Quaternion tf_quat;
        tf2::convert(quat, tf_quat);
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    }

    inline double angleBetweenVectors(const Eigen::Vector3d& a, const Eigen::Vector3d& b){
        return std::atan2(a.cross(b).norm(), a.dot(b));
    }

    inline Eigen::Vector3d computeCenter(const std::vector<Eigen::Vector3d> &points) {
        Eigen::Vector3d center(0.0, 0.0, 0.0);
        if (points.empty()) {
            return center; 
        }

        for (const auto &p : points) {
            center += p;
        }
        center /= static_cast<double>(points.size());

        return center;
    }

    inline Eigen::Vector3d computeStd(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &center) {
        Eigen::Vector3d stds(0.0, 0.0, 0.0);
        if (points.empty()) {
            return stds; 
        }

        for (const auto &p : points) {
            Eigen::Vector3d diff = p - center;
            stds(0) += diff(0) * diff(0);
            stds(1) += diff(1) * diff(1);
            stds(2) += diff(2) * diff(2);
        }
        stds /= static_cast<double>(points.size());
        stds = stds.array().sqrt(); 

        return stds;
    }

    // Oriented bounding box parsed from wall_detector marker messages
    struct WallOBB {
        Eigen::Vector3d center;
        Eigen::Matrix3d rotation;
        Eigen::Vector3d half_size;   // half extents in local frame

        bool contains(const Eigen::Vector3d& p) const {
            Eigen::Vector3d local = rotation.transpose() * (p - center);
            return (std::abs(local.x()) <= half_size.x()) &&
                   (std::abs(local.y()) <= half_size.y()) &&
                   (std::abs(local.z()) <= half_size.z());
        }
    };
}

#endif
