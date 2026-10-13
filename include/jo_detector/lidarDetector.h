/*
    FILE: lidarDetector.h
    ---------------------------------
    header file of lidar-based obstacle detector
*/
#ifndef JODETECTOR_LIDARDETECTOR_H
#define JODETECTOR_LIDARDETECTOR_H

#include <jo_detector/dbscan.h>
#include <jo_detector/utils.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>

namespace joDetector{
    struct Cluster
    {
        int cluster_id;
        Eigen::Vector4f centroid;
        pcl::PointCloud<pcl::PointXYZ>::Ptr points; // pointcloud

        // Geometry information
        Eigen::Vector3f dimensions;    // bbox size
        Eigen::Matrix3f eigen_vectors; // PCA eigen vectors
        Eigen::Vector3f eigen_values;  // PCA eigen values

        Cluster():
            cluster_id(-1),
            centroid(Eigen::Vector4f::Zero()),
            points(new pcl::PointCloud<pcl::PointXYZ>()) {}
    };

    class lidarDetector{
    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_; // current pointcloud
        std::vector<joDetector::Cluster> clusters_; // current cluster list
        std::vector<joDetector::box3D> bboxes_; // lidar bboxes

        // lidar DBSCAN parameters
        double eps_;
        int minPts_;
        double groundHeight_;
        double roofHeight_;

        // refinement parameters
        bool dbscanRefinementEnable_;
        double dbscanRefineMaxDiagonal_;
        double dbscanRefineMinDensity_;
        int dbscanRefineSplitMinPts_;
        double dbscanRefineSplitEps_;
        int dbscanRefineMinSubclusterPts_;
        double dbscanRefineAxisSliceWidth_;
        int dbscanRefineMaxDepth_;
        bool dbscanRefineRecursive_;
        double dbscanRefineMinBoxVolume_;

    public:
        lidarDetector();

        void setParams(
            double eps,
            int minPts,
            bool dbscanRefinementEnable,
            double dbscanRefineMaxDiagonal,
            double dbscanRefineMinDensity,
            int dbscanRefineSplitMinPts,
            double dbscanRefineSplitEps,
            int dbscanRefineMinSubclusterPts,
            double dbscanRefineAxisSliceWidth,
            int dbscanRefineMaxDepth,
            bool dbscanRefineRecursive,
            double dbscanRefineMinBoxVolume);

        void getPointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
        void lidarDBSCAN();
        std::vector<joDetector::Cluster>& getClusters();
        std::vector<joDetector::box3D>& getBBoxes();
    };
}

#endif