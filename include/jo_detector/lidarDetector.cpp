/*
    FILE: lidarDetector.cpp
    ------------------
    class function definitions for lidar-based obstacle detector
*/
#include <jo_detector/lidarDetector.h>

namespace joDetector{

    lidarDetector::lidarDetector(){
    }

    void lidarDetector::setParams(
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
        double dbscanRefineMinBoxVolume)
    {
        this->eps_ = eps;
        this->minPts_ = minPts;

        this->dbscanRefinementEnable_ = dbscanRefinementEnable;
        this->dbscanRefineMaxDiagonal_ = dbscanRefineMaxDiagonal;
        this->dbscanRefineMinDensity_ = dbscanRefineMinDensity;
        this->dbscanRefineSplitMinPts_ = dbscanRefineSplitMinPts;
        this->dbscanRefineSplitEps_ = dbscanRefineSplitEps;
        this->dbscanRefineMinSubclusterPts_ = dbscanRefineMinSubclusterPts;
        this->dbscanRefineAxisSliceWidth_ = dbscanRefineAxisSliceWidth;
        this->dbscanRefineMaxDepth_ = dbscanRefineMaxDepth;
        this->dbscanRefineRecursive_ = dbscanRefineRecursive;
        this->dbscanRefineMinBoxVolume_ = dbscanRefineMinBoxVolume;
    }

    void lidarDetector::getPointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
        this->cloud_ = cloud;
    }

    void lidarDetector::lidarDBSCAN(){
        this->clusters_.clear();
        this->bboxes_.clear();

        if (!cloud_ || cloud_->empty()){
            return;
        }

        std::vector<Point> points;
        points.reserve(cloud_->size());

        for (size_t i = 0; i < cloud_->size(); ++i){
            Point p;
            p.x = cloud_->points[i].x;
            p.y = cloud_->points[i].y;
            p.z = cloud_->points[i].z;
            p.clusterID = UNCLASSIFIED;
            points.push_back(p);
        }

        DBSCAN dbscan(minPts_, eps_, points);

        // refinement solo nel ramo lidar
        dbscan.setRefinementParams(
            this->dbscanRefinementEnable_,
            this->dbscanRefineMaxDiagonal_,
            this->dbscanRefineMinDensity_,
            this->dbscanRefineSplitMinPts_,
            this->dbscanRefineSplitEps_,
            this->dbscanRefineMinSubclusterPts_,
            this->dbscanRefineAxisSliceWidth_,
            this->dbscanRefineMaxDepth_,
            this->dbscanRefineRecursive_,
            this->dbscanRefineMinBoxVolume_);

        dbscan.run();

        std::vector<joDetector::Cluster> clustersTemp;
        std::vector<joDetector::box3D> bboxesTemp;

        if (this->dbscanRefinementEnable_){
            std::vector<joDetector::ClusterRefined> refinedClusters;
            dbscan.getRefinedClustersWithAABB(refinedClusters);

            clustersTemp.resize(refinedClusters.size());

            for (size_t i = 0; i < refinedClusters.size(); ++i){
                const auto& refined = refinedClusters[i];

                clustersTemp[i].cluster_id = static_cast<int>(i) + 1;

                for (size_t j = 0; j < refined.points.size(); ++j){
                    pcl::PointXYZ point;
                    point.x = refined.points[j].x();
                    point.y = refined.points[j].y();
                    point.z = refined.points[j].z();
                    clustersTemp[i].points->push_back(point);
                }

                if (!clustersTemp[i].points->empty()){
                    pcl::compute3DCentroid(*clustersTemp[i].points, clustersTemp[i].centroid);
                }

                clustersTemp[i].dimensions = Eigen::Vector3f(
                    refined.box.max_pt.x() - refined.box.min_pt.x(),
                    refined.box.max_pt.y() - refined.box.min_pt.y(),
                    refined.box.max_pt.z() - refined.box.min_pt.z());

                joDetector::box3D bbox;
                bbox.id = static_cast<int>(i);
                bbox.x = 0.5 * (refined.box.min_pt.x() + refined.box.max_pt.x());
                bbox.y = 0.5 * (refined.box.min_pt.y() + refined.box.max_pt.y());
                bbox.z = 0.5 * (refined.box.min_pt.z() + refined.box.max_pt.z());
                bbox.x_width = refined.box.max_pt.x() - refined.box.min_pt.x();
                bbox.y_width = refined.box.max_pt.y() - refined.box.min_pt.y();
                bbox.z_width = refined.box.max_pt.z() - refined.box.min_pt.z();
                bbox.Vx = 0.0;
                bbox.Vy = 0.0;

                bboxesTemp.push_back(bbox);
            }
        }
        else{
            int clusterNum = 0;
            for (size_t i = 0; i < dbscan.m_points.size(); ++i){
                const joDetector::Point& pDB = dbscan.m_points[i];
                if (pDB.clusterID > clusterNum){
                    clusterNum = pDB.clusterID;
                }
            }

            clustersTemp.resize(clusterNum);

            for (size_t i = 0; i < dbscan.m_points.size(); ++i){
                if (dbscan.m_points[i].clusterID > 0){
                    pcl::PointXYZ point;
                    point.x = dbscan.m_points[i].x;
                    point.y = dbscan.m_points[i].y;
                    point.z = dbscan.m_points[i].z;
                    clustersTemp[dbscan.m_points[i].clusterID - 1].points->push_back(point);
                    clustersTemp[dbscan.m_points[i].clusterID - 1].cluster_id = dbscan.m_points[i].clusterID;
                }
            }

            for (size_t i = 0; i < clustersTemp.size(); ++i){
                auto& cluster = clustersTemp[i];

                if (cluster.points->empty()){
                    continue;
                }

                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*cluster.points, centroid);
                cluster.centroid = centroid;

                pcl::PointXYZ minPt, maxPt;
                pcl::getMinMax3D(*cluster.points, minPt, maxPt);

                cluster.dimensions = Eigen::Vector3f(
                    maxPt.x - minPt.x,
                    maxPt.y - minPt.y,
                    maxPt.z - minPt.z);

                joDetector::box3D bbox;
                bbox.id = static_cast<int>(i);
                bbox.x = 0.5 * (minPt.x + maxPt.x);
                bbox.y = 0.5 * (minPt.y + maxPt.y);
                bbox.z = 0.5 * (minPt.z + maxPt.z);
                bbox.x_width = maxPt.x - minPt.x;
                bbox.y_width = maxPt.y - minPt.y;
                bbox.z_width = maxPt.z - minPt.z;
                bbox.Vx = 0.0;
                bbox.Vy = 0.0;

                bboxesTemp.push_back(bbox);
            }
        }

        this->clusters_ = clustersTemp;
        this->bboxes_ = bboxesTemp;
    }

    std::vector<joDetector::Cluster>& lidarDetector::getClusters(){
        return this->clusters_;
    }

    std::vector<joDetector::box3D>& lidarDetector::getBBoxes(){
        return this->bboxes_;
    }
}