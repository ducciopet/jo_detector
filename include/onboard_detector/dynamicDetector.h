/*
    FILE: dynamicDetector.h
    ---------------------------------
    header file of dynamic obstacle detector
*/
#ifndef ONBOARDDETECTOR_DYNAMICDETECTOR_H
#define ONBOARDDETECTOR_DYNAMICDETECTOR_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <onboard_detector/dbscan.h>
#include <onboard_detector/uvDetector.h>
#include <onboard_detector/lidarDetector.h>
#include <onboard_detector/kalmanFilter.h>
#include <onboard_detector/utils.h>
#include <onboard_detector/srv/get_dynamic_obstacles.hpp>
#include <jo_msgs/msg/obstacle.hpp>
#include <jo_msgs/msg/obstacle_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <random>
#include <mutex>
#include <thread>
#include <unordered_set>

namespace onboardDetector{

    class dynamicDetector{

    private:
        std::string ns_;
        std::string hint_;

        // ROS node
        rclcpp::Node::SharedPtr nh_;
        std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
        std::shared_ptr<tf2_ros::TransformListener> tfListener_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depthSub_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> lidarCloudSub_;
        std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> poseSub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, geometry_msgs::msg::PoseStamped> depthPoseSync;
        std::shared_ptr<message_filters::Synchronizer<depthPoseSync>> depthPoseSync_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, geometry_msgs::msg::PoseStamped> lidarPoseSync;
        std::shared_ptr<message_filters::Synchronizer<lidarPoseSync>> lidarPoseSync_;
        std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odomSub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, nav_msgs::msg::Odometry> depthOdomSync;
        std::shared_ptr<message_filters::Synchronizer<depthOdomSync>> depthOdomSync_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry> lidarOdomSync;
        std::shared_ptr<message_filters::Synchronizer<lidarOdomSync>> lidarOdomSync_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr colorImgSub_;
        rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr yoloDetectionSub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr groundHeightSub_;
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr wallMarkersSub_;

        // Wall bounding boxes received from wall_detector
        std::vector<WallOBB> wallBBoxes_;
        std::mutex wallBBoxesMutex_;

        rclcpp::TimerBase::SharedPtr mainTimer_;
        rclcpp::TimerBase::SharedPtr visTimer_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr uvDepthMapPub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr uDepthMapPub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr uvBirdViewPub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detectedColorImgPub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr uvBBoxesFilteredPub_;        
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr uvBBoxesPub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dbBBoxesFilteredPub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dbBBoxesPub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualBBoxesPub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lidarBBoxesFilteredPub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lidarBBoxesPub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr filteredBBoxesBeforeYoloPub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr filteredBBoxesPub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trackedBBoxesPub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr predictedBBoxesActivePub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr predictedBBoxesMissedPub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr predictedBBoxesUnconfirmedPub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dynamicBBoxesPub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr potentiallyDynamicBBoxesPub_;
        rclcpp::Publisher<jo_msgs::msg::ObstacleArray>::SharedPtr trackedObstaclesPub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trackedObstaclesBBoxesPub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filteredDepthPointsPub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidarClustersPub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filteredPointsPub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamicPointsPub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rawDynamicPointsPub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downSamplePointsPub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rawLidarPointsPub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filteredLidarVelodynePub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr yoloPointsPub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr historyTrajPub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr velVisPub_;
        rclcpp::Service<onboard_detector::srv::GetDynamicObstacles>::SharedPtr getDynamicObstacleServer_;

        // DETECTOR
        std::shared_ptr<onboardDetector::UVdetector> uvDetector_;
        std::shared_ptr<onboardDetector::DBSCAN> dbCluster_;
        std::shared_ptr<onboardDetector::lidarDetector> lidarDetector_;

        // SENSOR INFO
        // CAMERA DEPTH
        double fx_, fy_, cx_, cy_; // depth camera intrinsics
        double depthScale_; // value / depthScale
        double depthMinValue_, depthMaxValue_;
        double raycastMaxLength_;
        int depthFilterMargin_, skipPixel_; // depth filter margin
        int imgCols_, imgRows_;

        // CAMERA COLOR
        double fxC_, fyC_, cxC_, cyC_;

        bool lidarToDepthCamOk_ = false;

        // Cached static TFs: looked up once (they don't change)
        Eigen::Matrix4d T_imu_base_;      // imu_link ← base_link
        Eigen::Matrix4d T_base_velodyne_; // base_link ← velodyne
        bool staticTfsCached_ = false;

        // PARAMETETER
        // Topics
        int localizationMode_;
        std::string depthTopicName_;
        std::string colorImgTopicName_;
        std::string lidarTopicName_;
        std::string poseTopicName_;
        std::string odomTopicName_;
        bool useTfPose_;
        std::string tfOdomFrame_;
        std::string tfLidarFrame_;
        std::string tfDepthFrame_;
        std::string tfColorFrame_;

        // System
        double dt_;

        // DBSCAN Common
        double groundHeight_;
        double roofHeight_;
        double groundRoofOffset_;
        double trackMaxZminAboveGround_;  // bboxes with zmin > groundHeight_ + this are not tracked
        bool groundEstimated_;

                
        // DBSCAN visual param
        double voxelOccThresh_;
        double voxelSize_;
        int dbMinPointsCluster_;
        double dbEpsilon_;
        
        // DBSCAN LiDAR param
        int lidarDBMinPoints_;
        double lidarDBEpsilon_;
        int gaussianDownSampleRate_;
        int downSampleThresh_;
        
        // DBSCAN refinement param
        bool dbscanRefinementEnable_;
        bool visualDbscanRefinementEnable_;
        double dbscanRefineMaxDiagonal_;
        double dbscanRefineMinDensity_;
        int dbscanRefineSplitMinPts_;
        double dbscanRefineSplitEps_;
        int dbscanRefineMinSubclusterPts_;
        double dbscanRefineAxisSliceWidth_;
        int dbscanRefineMaxDepth_;
        bool dbscanRefineRecursive_;
        double dbscanRefineMinBoxVolume_;

        // UV detection param 
        double uvBoxMaxDiagonal_;
        double boxMaxHeight_;
        double boxMaxVolume_;

        // LiDAR Visual Filtering
        double visualboxIOUThresh_;
        double visualboxIOVThresh_;
        double lidarVisualboxIOUThresh_;
        double lidarVisualboxIOVThresh_;
        double samegroupIOUThresh_;
        double samegroupIOVThresh_;
        std::string visualmergingFlag_;
        std::string lidarVisualmergingFlag_;
        bool lidarVisualMergerLeafOnly_;
        bool uvUnmergedFlag_;
        bool dbUnmergedFlag_;
        bool lidarUnmergedFlag_;
        bool visualUnmergedFlag_;
        bool finalMergeFlag_;

        // Tracking and data association
        int nextTrackId_ = 0;
        double maxMatchRange_;
        int histSize_;
        int kfAvgFrames_;

        // Kalman Filter V2 parameters (position-only observation)
        double eP_v2_;
        double eQPos_v2_;
        double eQVel_v2_;
        double eQAcc_v2_;
        double eRPos_v2_;
        int predictionVisualizationSteps_;

        // (coastingMaxMissedFrames_ removed: predicted bboxes are never put in output)

        std::vector<int> missedFrames_;
        int maxMissedFrames_;
        int maxMissedFramesYolo_;    // extended lifetime for YOLO-confirmed tracks
        std::vector<int> hitStreak_;
        std::vector<int> trackAge_;
        std::vector<bool> confirmedTracks_;
        std::vector<int> staticStreak_;  // consecutive frames a confirmed track has been stationary
        std::vector<int> nonYoloInFovStreak_;  // consecutive frames inside FOV matched to non-YOLO detection
        int maxNonYoloInFovFrames_;  // after this many frames, clear sticky is_yolo_candidate
        std::vector<double> yoloXWidth_;  // x_width of the YOLO detection that created the YOLO flag
        std::vector<double> yoloYWidth_;  // y_width of the YOLO detection that created the YOLO flag
        std::vector<int> yoloBaseMismatchStreak_;  // consecutive frames where base size differs from original
        int maxYoloBaseMismatchFrames_;  // after this many frames, clear sticky is_yolo_candidate
        double yoloBaseMismatchThresh_;  // relative diff threshold to count as mismatch

        double minMatchScore_;

        double maxMatchSpeed_;
        double matchPosScoreWeight_;
        double matchSizeScoreWeight_;
        double matchIou2DScoreWeight_;
        double maxRelativeSizeDiffMatch_;
        double matchVelocityDirectionScoreWeight_;
        double matchYoloClassConsistencyWeight_; // penalty when yolo track matches non-yolo detection

        double duplicateTrackDistThresh_;
        double duplicateTrackIou2DThresh_;
        double duplicateSizeRelThresh_;

        bool enableTrackingDebugLogs_;

        double matchPrevObsPosScoreWeight_;
        double matchPrevObsIou2DScoreWeight_;

        // Environment
        bool isIndoor_;

        // YOLO Dynamic Classification
        std::vector<std::string> yoloDynamicClasses_;
        double yoloPointFractionThresh_;
        double yoloSparseLargeBoxVolumeThresh_;  // [m^3] boxes above this need enough YOLO points to avoid sparse-large false positives
        int yoloSparseLargeMinPoints_;           // min YOLO foreground points required for large boxes
        double yoloSparseLargeMinDensity_;       // [pts/m^3] min YOLO point density required for large boxes
        double yoloMaxHeightRatio_;              // max allowed ratio of 3D bbox z_width to YOLO points Z span
        double yoloMinHeightSpan_;               // [m] minimum YOLO Z span (clamp to avoid near-zero division)
        double yoloDepthTolerance_;
        double yoloOutdoorDepthTolerance_;     // [m] depth window beyond the 20th-percentile foreground depth (outdoor filter)
        double yoloHeightCorrectionThreshold_; // [m] min height diff to override 3D box height with YOLO-derived height
        double yolo2dIouThreshold_;            // min 2D IoU between projected 3D bbox and YOLO rect to assign candidate (no-depth fallback)
        bool yoloXYResize_;                    // if true, resize x/y of 3D bbox to fit YOLO foreground points

        // Track confirmation / natural motion
        int minConfirmHits_;
        double minNaturalMotionDist_;
        double maxNaturalMotionDist_;
        double maxNaturalInnovation_;
        double maxVelocityDirectionErrorConfirm_;
        double maxVelocityDirectionErrorTracked_;
        double stationarySpeedThresh_;
        bool trackSteadyObjects_;
        double confirmedTrackAssocBonus_;
        double yoloTrackAssocBonus_;
        double maxNaturalInnovationConfirmed_;
        double maxNaturalInnovationDynamic_;
        double maxVelocityDirectionErrorConfirmDynamic_;
        double maxVelocityDirectionErrorTrackedDynamic_;
        double minMatchScoreConfirmed_;
        double minMatchScoreDynamic_;
        int minConfirmHitsOutsideFov_;
        double minOutsideFovObsSpeed_;
        double maxOutsideFovObsSpeed_;
        double maxNaturalInnovationOutsideFov_;
        double maxVelocityDirectionErrorConfirmOutsideFov_;
        int outsideFovObsVelWindow_;
        double minOutsideFovAvgObsSpeed_;
        double maxOutsideFovAvgObsSpeed_;
        double maxMatchRangeOutsideFov_;
        double maxMatchSpeedOutsideFov_;
        double maxRelativeSizeDiffOutsideFov_;
        double outsideFovConfirmedAssocBonus_;
        double outsideFovTentativeAssocBonus_;
        double outsideFovTurnMaxPosDist_;
        double outsideFovTurnMaxSpeed_;
        double outsideFovTurnMaxSizeDiff_;
        double outsideFovTurnMinObsSpeed_;
        double outsideFovTurnAssocBonus_;
        int outsideFovTurnMinTrackAge_;
        int outsideFovClassWindow_;
        int outsideFovClassMinTrackAge_;
        double outsideFovClassMinNetDisp_;
        double outsideFovClassMinNetSpeed_;
        double outsideFovClassMinStraightness_;
        double outsideFovClassMaxStepSpeed_;
    
        // Classification
        int skipFrame_;
        double dynaVelThresh_;
        double dynaVoteThresh_;
        double dynaKfVelStdRatio_; // max std/mean ratio for Kalman velocity to be considered confident
        int forceDynaFrames_;
        int forceDynaCheckRange_;
        int dynamicConsistThresh_;

        // Constrain size
        bool constrainSize_;
        std::vector<Eigen::Vector3d> targetObjectSize_; 
        Eigen::Vector3d maxObjectSize_; 

        // SENSOR DATA
        cv::Mat depthImage_;
        Eigen::Vector3d position_; // robot position
        Eigen::Matrix3d orientation_; // robot orientation
        Eigen::Vector3d positionDepth_; // depth camera position
        Eigen::Matrix3d orientationDepth_; // depth camera orientation
        Eigen::Vector3d positionColor_; // color camera position
        Eigen::Matrix3d orientationColor_; // color camera orientation
        Eigen::Vector3d positionLidar_; // color camera position
        Eigen::Matrix3d orientationLidar_; // color camera orientation
        bool hasSensorPose_;
        Eigen::Vector3d localSensorRange_;
        Eigen::Vector3d localLidarRange_;

        //LIDAR DATA
        sensor_msgs::msg::PointCloud2::ConstSharedPtr latestCloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr lidarCloud_ = nullptr; 
        std::vector<onboardDetector::Cluster> lidarClusters_;

        // DETECTOR DATA
        std::vector<onboardDetector::box3D> uvBBoxes_; // uv detector bounding boxes
        int projPointsNum_ = 0;
        std::vector<Eigen::Vector3d> projPoints_; // projected points from depth image
        std::vector<double> pointsDepth_;
        std::vector<Eigen::Vector3d> filteredDepthPoints_; // filtered point cloud data
        std::vector<onboardDetector::box3D> dbBBoxes_; // DBSCAN bounding boxes
        std::vector<std::vector<Eigen::Vector3d>> pcClustersVisual_; // pointcloud clusters
        std::vector<Eigen::Vector3d> pcClusterCentersVisual_; // pointcloud cluster centers
        std::vector<Eigen::Vector3d> pcClusterStdsVisual_; // pointcloud cluster standard deviation in each axis      
        std::vector<onboardDetector::box3D> filteredBBoxesBeforeYolo_; // filtered bboxes before yolo
        std::vector<onboardDetector::box3D> filteredBBoxes_; // filtered bboxes
        std::vector<std::vector<Eigen::Vector3d>> filteredPcClusters_; // pointcloud clusters after filtering by UV and DBSCAN fusion
        std::vector<Eigen::Vector3d> filteredPcClusterCenters_; // filtered pointcloud cluster centers
        std::vector<Eigen::Vector3d> filteredPcClusterStds_; // filtered pointcloud cluster standard deviation in each axis
        std::vector<onboardDetector::box3D> visualBBoxes_; // visual bobxes detected by camera
        std::vector<onboardDetector::box3D> lidarBBoxes_; // bboxes detected by lidar (have static and dynamic)
        std::vector<onboardDetector::box3D> trackedBBoxes_; // bboxes tracked from kalman filtering
        std::vector<onboardDetector::box3D> predictedBBoxesActive_;       // confirmed + matched last frame
        std::vector<onboardDetector::box3D> predictedBBoxesMissed_;       // confirmed + coasting (no match)
        std::vector<onboardDetector::box3D> predictedBBoxesUnconfirmed_;  // not yet confirmed
        std::vector<onboardDetector::box3D> dynamicBBoxes_;            // boxes classified as dynamic
        std::vector<onboardDetector::box3D> potentiallyDynamicBBoxes_; // YOLO-identified but KF speed ~ 0

        // TRACKING AND ASSOCIATION DATA
        bool newDetectFlag_;
        std::vector<std::deque<onboardDetector::box3D>> boxHist_; // data association result: history of filtered bounding boxes for each box in current frame
        std::vector<std::deque<std::vector<Eigen::Vector3d>>> pcHist_; // data association result: history of filtered pc clusteres for each pc cluster in current frame
        std::vector<std::deque<Eigen::Vector3d>> pcCenterHist_; 
        std::vector<onboardDetector::kalman_filter> filters_; // kalman filter for each objects

        // YOLO RESULTS
        vision_msgs::msg::Detection2DArray yoloDetectionResults_; // yolo detected 2D results
        cv::Mat detectedColorImage_;

    public:

        dynamicDetector();
        dynamicDetector(const rclcpp::Node::SharedPtr& nh);
        void initDetector(const rclcpp::Node::SharedPtr& nh);

        void initParam();
        void registerPub();
        void registerCallback();

        // service
        bool getDynamicObstacles(const std::shared_ptr<onboard_detector::srv::GetDynamicObstacles::Request> req,
                     std::shared_ptr<onboard_detector::srv::GetDynamicObstacles::Response> res);

        // callback
        void depthPoseCB(const sensor_msgs::msg::Image::ConstSharedPtr& img, const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose);
        void depthOdomCB(const sensor_msgs::msg::Image::ConstSharedPtr& img, const nav_msgs::msg::Odometry::ConstSharedPtr& odom);
        void lidarPoseCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloudMsg, const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose);
        void lidarOdomCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloudMsg, const nav_msgs::msg::Odometry::ConstSharedPtr& odom);
        void colorImgCB(const sensor_msgs::msg::Image::ConstSharedPtr& img);
        void yoloDetectionCB(const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections);
        void groundHeightCB(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg);
        void wallMarkersCB(const visualization_msgs::msg::MarkerArray::ConstSharedPtr& msg);
        void mainCB();
        void visCB();

        // detect function
        void uvDetect();
        void dbscanDetect();
        void lidarDetect();
        void filterLVBBoxes(); // filter lidar and vision bounding boxes
        void transformUVBBoxes(std::vector<onboardDetector::box3D>& bboxes);
        
        // Visual DBSCAN Detector Functions
        void projectDepthImage();
        void filterPoints(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& filteredPoints);
        void clusterPointsAndBBoxes(const std::vector<Eigen::Vector3d>& points, std::vector<onboardDetector::box3D>& bboxes, std::vector<std::vector<Eigen::Vector3d>>& pcClusters, std::vector<Eigen::Vector3d>& pcClusterCenters, std::vector<Eigen::Vector3d>& pcClusterStds);
        void voxelFilter(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& filteredPoints);
        
        // detection helper functions
        void calcPcFeat(const std::vector<Eigen::Vector3d>& pcCluster, Eigen::Vector3d& pcClusterCenter, Eigen::Vector3d& pcClusterStd);
        double calBoxIOU(const onboardDetector::box3D& box1, const onboardDetector::box3D& box2, bool ignoreZmin=false);
        double calBoxIOV(const onboardDetector::box3D& box1, const onboardDetector::box3D& box2, bool ignoreZmin=false);

        // Data association and tracking functions
        void boxAssociation(std::vector<int>& bestMatch);
        void boxAssociationHelper(std::vector<int>& bestMatch);
        void BboxesMerger(const std::vector<onboardDetector::box3D>& group1BBoxes_, const std::vector<onboardDetector::box3D>& group2BBoxes_, const std::vector<std::vector<Eigen::Vector3d>>& group1pcClusters_, const std::vector<Eigen::Vector3d>& group1pcClusterCenters_, const std::vector<Eigen::Vector3d>& group1pcClusterStds_, const std::vector<std::vector<Eigen::Vector3d>>& group2pcClusters_, const std::vector<Eigen::Vector3d>& group2pcClusterCenters_, const std::vector<Eigen::Vector3d>& group2pcClusterStds_, std::vector<onboardDetector::box3D>& BBoxesTemp, std::vector<std::vector<Eigen::Vector3d>>& PcClustersTemp, std::vector<Eigen::Vector3d>& PcClusterCentersTemp, std::vector<Eigen::Vector3d>& PcClusterStdsTemp, std::string merging_style, bool flag_group1, bool flag_group2, double boxIOUThresh_, double boxIOVThresh_, bool leaf_only = false);
        void mergeNestedGroup(const std::vector<onboardDetector::box3D>& inBoxes, const std::vector<std::vector<Eigen::Vector3d>>& inClusters, const std::vector<Eigen::Vector3d>& inCenters, const std::vector<Eigen::Vector3d>& inStds,     std::vector<onboardDetector::box3D>& outBoxes, std::vector<std::vector<Eigen::Vector3d>>& outClusters, std::vector<Eigen::Vector3d>& outCenters, std::vector<Eigen::Vector3d>& outStds);
        void mergeBoxesSet(const std::vector<onboardDetector::box3D>& boxes, const std::vector<std::vector<Eigen::Vector3d>>& clusters, const std::vector<int>& indices, onboardDetector::box3D& outBox, std::vector<Eigen::Vector3d>& outCluster, Eigen::Vector3d& center, Eigen::Vector3d& stddev);
        void findBestMatch(const std::vector<onboardDetector::box3D>& predictedBBoxes, const std::vector<onboardDetector::box3D>& previousObservedBBoxes, std::vector<int>& bestMatch);
        void kalmanFilterAndUpdateHist(const std::vector<int>& bestMatch);
        void kalmanFilterMatrixAccV2(const onboardDetector::box3D& currDetectedBBox, MatrixXd& states, MatrixXd& A, MatrixXd& B, MatrixXd& H, MatrixXd& P, MatrixXd& Q, MatrixXd& R);
        void getKalmanObservationAccV2(const onboardDetector::box3D& currDetectedBBox, MatrixXd& Z);
        double computeBoxIoU2D(const onboardDetector::box3D& boxA, const onboardDetector::box3D& boxB) const;
        void getPredictedBBoxesFromFilters(std::vector<onboardDetector::box3D>& propedBBoxes, std::vector<Eigen::Vector3d>& propedPcCenters);
        void populatePredictionVisualizationBBoxes();
        bool isCloseToExistingTrack(const onboardDetector::box3D& currDetectedBBox, const Eigen::Vector3d& currPcCenter, const std::vector<bool>& prevMatched);
        double computeRelativeSizeDiff(const onboardDetector::box3D& a, const onboardDetector::box3D& b) const;
        bool areBoxesDuplicate2D(const onboardDetector::box3D& a, const onboardDetector::box3D& b) const;
        bool isDuplicateOfTrackedThisFrame(const onboardDetector::box3D& currDetectedBBox, const std::vector<onboardDetector::box3D>& trackedBBoxesTemp, int& duplicateTrackId) const;
        bool isDetectionDuplicateOfPrediction(const onboardDetector::box3D& currDetectedBBox, const onboardDetector::box3D& predictedBox, double& centerDist, double& iou2d, double& relSizeDiff) const;
        double computeAssociationScore(const onboardDetector::box3D& predictedBox, const onboardDetector::box3D& previousObservedBox, const onboardDetector::box3D& currentBox, double dt, double& predPosDist, double& requiredSpeed, double& relSizeDiff, double& predIou2d, double& prevObsPosDist, double& prevObsIou2d, std::string& rejectReason) const;
        double clampPositive(double value, double minValue) const;
        void getPreviousObservedBBoxes(std::vector<onboardDetector::box3D>& previousObservedBBoxes) const;
        bool tryAugmentDetectionMatch(int detIdx, const std::vector<std::vector<int>>& candidatePrevIdxByCurr, std::vector<int>& assignedPrevToCurr, std::vector<int>& assignedCurrToPrev, std::vector<int>& visitTokenByPrev, int visitToken) const;
        void assignMatchesGlobally(const std::vector<onboardDetector::matchCandidate>& candidates, int numCurr, int numPrev, std::vector<int>& bestMatch) const;
        bool isNaturalMotion(int trackIdx, const onboardDetector::box3D& currDetectedBBox) const;
        bool shouldConfirmTrack(int trackIdx, const onboardDetector::box3D& currDetectedBBox, int newHitStreak) const;
        double computeVelocityDirectionError(int trackIdx, const onboardDetector::box3D& currDetectedBBox) const;
        bool passesVelocityDirectionGate(int trackIdx, const onboardDetector::box3D& currDetectedBBox, bool alreadyConfirmed, bool yoloExempt = false) const;
        bool isTrackConfirmedByIdx(int trackIdx) const;
        double getAdaptiveMaxInnovation(int trackIdx, const onboardDetector::box3D& currDetectedBBox) const;
        double getAdaptiveMinMatchScore(int trackIdx, const onboardDetector::box3D& currentBox) const;
        double computeObservedSpeedOverWindow(int trackIdx, const onboardDetector::box3D& currDetectedBBox, int window) const;
        bool isLidarOnlyOutsideFovAssociation(const onboardDetector::box3D& predictedBox, const onboardDetector::box3D& currentBox) const;
        bool isAbruptTurnAllowedOutsideFov(int trackIdx, const onboardDetector::box3D& predictedBox, const onboardDetector::box3D& currentBox, double dt) const;
        bool hasConsistentObservedMotionOutsideFov(int trackIdx, int window, double& netDisp, double& netSpeed, double& pathLen, double& straightness, double& maxStepSpeed) const;
        
        // visualization
        void getDynamicPc(std::vector<Eigen::Vector3d>& dynamicPc);
        void publishUVImages(); 
        void publishColorImages();
        void publishPoints(const std::vector<Eigen::Vector3d>& points, const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher);
        void publish3dBox(const std::vector<onboardDetector::box3D>& bboxes, const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher, double r, double g, double b, double a = 1.0, bool highlightYolo = false, bool useClassificationColors = true, double lifetimeSeconds = -1.0);
        void publishHistoryTraj();
        void publishVelVis();
        void publishLidarClusters();
        void publishFilteredPoints();
        void publishRawDynamicPoints();
        void publishDynamicObstacleArray();
        void publishTrackedDynamicObstacleMarkers();
        void refreshTrackedBBoxClassificationFlags();

        // helper function
        void transformBBox(const Eigen::Vector3d& center, const Eigen::Vector3d& size, const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation, Eigen::Vector3d& newCenter, Eigen::Vector3d& newSize);

        // user functions
        void getDynamicObstacles(std::vector<onboardDetector::box3D>& incomeDynamicBBoxes, const Eigen::Vector3d &robotSize = Eigen::Vector3d(0.0,0.0,0.0));
        // inline helper functions
        bool isInFilterRange(const Eigen::Vector3d& pos);
        bool isInCameraFOV(const Eigen::Vector3d& worldPoint) const;
        bool isInsideAnyWall(const Eigen::Vector3d& pos) const;
        void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx, double res);
        int indexToAddress(const Eigen::Vector3i& idx, double res);
        int posToAddress(const Eigen::Vector3d& pos, double res);
        void indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pos, double res);
        void getCameraPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose, Eigen::Matrix4d& camPoseDepthMatrix, Eigen::Matrix4d& camPoseColorMatrix);
        void getCameraPose(const nav_msgs::msg::Odometry::ConstSharedPtr& odom, Eigen::Matrix4d& camPoseDepthMatrix, Eigen::Matrix4d& camPoseColorMatrix);
        void getLidarPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose, Eigen::Matrix4d& lidarPoseMatrix);
        void getLidarPose(const nav_msgs::msg::Odometry::ConstSharedPtr& odom, Eigen::Matrix4d& lidarPoseMatrix);
        bool lookupTfMatrix(const std::string& targetFrame, const std::string& sourceFrame, Eigen::Matrix4d& transformMatrix);
        bool ensureStaticTfs();
        onboardDetector::Point eigenToDBPoint(const Eigen::Vector3d& p);
        Eigen::Vector3d dbPointToEigen(const onboardDetector::Point& pDB);
        void eigenToDBPointVec(const std::vector<Eigen::Vector3d>& points, std::vector<onboardDetector::Point>& pointsDB, int size);       
        bool isOversizedBBox(const onboardDetector::box3D& bbox) const;
    };


    inline bool dynamicDetector::isInFilterRange(const Eigen::Vector3d& pos){
        if ((pos(0) >= this->position_(0) - this->localSensorRange_(0)) and (pos(0) <= this->position_(0) + this->localSensorRange_(0)) and 
            (pos(1) >= this->position_(1) - this->localSensorRange_(1)) and (pos(1) <= this->position_(1) + this->localSensorRange_(1)) and 
            (pos(2) >= this->position_(2) - this->localSensorRange_(2)) and (pos(2) <= this->position_(2) + this->localSensorRange_(2))){
            return true;
        }
        else{
            return false;
        }        
    }

    inline bool dynamicDetector::isInCameraFOV(const Eigen::Vector3d& worldPoint) const {
        // Transform world point into the depth camera frame
        Eigen::Vector3d ptCam = orientationDepth_.transpose() * (worldPoint - positionDepth_);
        // ptCam.z() is depth along the camera optical axis
        if (ptCam.z() < depthMinValue_ || ptCam.z() > depthMaxValue_) return false;
        // Project onto image plane and check pixel bounds
        double u = fx_ * ptCam.x() / ptCam.z() + cx_;
        double v = fy_ * ptCam.y() / ptCam.z() + cy_;
        return (u >= 0.0 && u < imgCols_ && v >= 0.0 && v < imgRows_);
    }

    inline bool dynamicDetector::isInsideAnyWall(const Eigen::Vector3d& pos) const {
        for (const auto& wall : wallBBoxes_) {
            if (wall.contains(pos)) return true;
        }
        return false;
    }

    inline void dynamicDetector::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx, double res){
        idx(0) = floor( (pos(0) - this->position_(0) + localSensorRange_(0)) / res);
        idx(1) = floor( (pos(1) - this->position_(1) + localSensorRange_(1)) / res);
        idx(2) = floor( (pos(2) - this->position_(2) + localSensorRange_(2)) / res);
    }

    inline int dynamicDetector::indexToAddress(const Eigen::Vector3i& idx, double res){
        return idx(0) * ceil(2*this->localSensorRange_(1)/res) * ceil(2*this->localSensorRange_(2)/res) + idx(1) * ceil(2*this->localSensorRange_(2)/res) + idx(2);
        // return idx(0) * ceil(this->localSensorRange_(0)/res) + idx(1) * ceil(this->localSensorRange_(1)/res) + idx(2);
    }

    inline int dynamicDetector::posToAddress(const Eigen::Vector3d& pos, double res){
        Eigen::Vector3i idx;
        this->posToIndex(pos, idx, res);
        return this->indexToAddress(idx, res);
    }

    inline void dynamicDetector::indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pos, double res){
		pos(0) = (idx(0) + 0.5) * res - localSensorRange_(0) + this->position_(0);
		pos(1) = (idx(1) + 0.5) * res - localSensorRange_(1) + this->position_(1);
		pos(2) = (idx(2) + 0.5) * res - localSensorRange_(2) + this->position_(2);
	}
    
    inline void dynamicDetector::getCameraPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose, Eigen::Matrix4d& camPoseDepthMatrix, Eigen::Matrix4d& camPoseColorMatrix){
        if (!ensureStaticTfs()) {
            camPoseDepthMatrix.setIdentity();
            camPoseColorMatrix.setIdentity();
            return;
        }
        // pose is odom → imu_link (from GLIM)
        Eigen::Quaterniond quat(pose->pose.orientation.w, pose->pose.orientation.x,
                                pose->pose.orientation.y, pose->pose.orientation.z);
        Eigen::Matrix4d T_odom_imu;
        T_odom_imu.setIdentity();
        T_odom_imu.block<3,3>(0,0) = quat.toRotationMatrix();
        T_odom_imu(0,3) = pose->pose.position.x;
        T_odom_imu(1,3) = pose->pose.position.y;
        T_odom_imu(2,3) = pose->pose.position.z;

        // chain: odom ← imu_link ← base_link ← velodyne
        Eigen::Matrix4d T_odom_velodyne = T_odom_imu * T_imu_base_ * T_base_velodyne_;

        Eigen::Matrix4d lidarToDepthCam, lidarToColorCam;
        this->lidarToDepthCamOk_ = this->lookupTfMatrix(this->tfLidarFrame_, this->tfDepthFrame_, lidarToDepthCam);
        bool lidarToColorCamOk = this->lookupTfMatrix(this->tfLidarFrame_, this->tfColorFrame_, lidarToColorCam);

        if (this->lidarToDepthCamOk_ && lidarToColorCamOk) {
            camPoseDepthMatrix = T_odom_velodyne * lidarToDepthCam;
            camPoseColorMatrix = T_odom_velodyne * lidarToColorCam;
            return;
        }
        camPoseDepthMatrix.setIdentity();
        camPoseColorMatrix.setIdentity();
    }

    inline void dynamicDetector::getCameraPose(const nav_msgs::msg::Odometry::ConstSharedPtr& odom, Eigen::Matrix4d& camPoseDepthMatrix, Eigen::Matrix4d& camPoseColorMatrix){
        if (!ensureStaticTfs()) {
            camPoseDepthMatrix.setIdentity();
            camPoseColorMatrix.setIdentity();
            return;
        }
        // odom is odom → base_link (from EKF on /odometry/filtered)
        Eigen::Quaterniond quat(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
        Eigen::Matrix4d T_odom_base;
        T_odom_base.setIdentity();
        T_odom_base.block<3,3>(0,0) = quat.toRotationMatrix();
        T_odom_base(0,3) = odom->pose.pose.position.x;
        T_odom_base(1,3) = odom->pose.pose.position.y;
        T_odom_base(2,3) = odom->pose.pose.position.z;

        // chain: odom ← base_link ← velodyne
        Eigen::Matrix4d T_odom_velodyne = T_odom_base * T_base_velodyne_;

        Eigen::Matrix4d lidarToDepthCam, lidarToColorCam;
        this->lidarToDepthCamOk_ = this->lookupTfMatrix(this->tfLidarFrame_, this->tfDepthFrame_, lidarToDepthCam);
        bool lidarToColorCamOk = this->lookupTfMatrix(this->tfLidarFrame_, this->tfColorFrame_, lidarToColorCam);

        if (this->lidarToDepthCamOk_ && lidarToColorCamOk) {
            camPoseDepthMatrix = T_odom_velodyne * lidarToDepthCam;
            camPoseColorMatrix = T_odom_velodyne * lidarToColorCam;
            return;
        }
        camPoseDepthMatrix.setIdentity();
        camPoseColorMatrix.setIdentity();
    }

    inline void dynamicDetector::getLidarPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose, Eigen::Matrix4d& lidarPoseMatrix){
        if (!ensureStaticTfs()) {
            lidarPoseMatrix.setIdentity();
            return;
        }
        // pose is odom → imu_link (from GLIM)
        Eigen::Quaterniond quat(pose->pose.orientation.w, pose->pose.orientation.x,
                                pose->pose.orientation.y, pose->pose.orientation.z);
        Eigen::Matrix4d T_odom_imu;
        T_odom_imu.setIdentity();
        T_odom_imu.block<3,3>(0,0) = quat.toRotationMatrix();
        T_odom_imu(0,3) = pose->pose.position.x;
        T_odom_imu(1,3) = pose->pose.position.y;
        T_odom_imu(2,3) = pose->pose.position.z;
        // chain: odom ← imu_link ← base_link ← velodyne
        lidarPoseMatrix = T_odom_imu * T_imu_base_ * T_base_velodyne_;
    }

    inline void dynamicDetector::getLidarPose(const nav_msgs::msg::Odometry::ConstSharedPtr& odom, Eigen::Matrix4d& lidarPoseMatrix){
        if (!ensureStaticTfs()) {
            lidarPoseMatrix.setIdentity();
            return;
        }
        // odom is odom → base_link (from EKF on /odometry/filtered)
        Eigen::Quaterniond quat(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
        Eigen::Matrix4d T_odom_base;
        T_odom_base.setIdentity();
        T_odom_base.block<3,3>(0,0) = quat.toRotationMatrix();
        T_odom_base(0,3) = odom->pose.pose.position.x;
        T_odom_base(1,3) = odom->pose.pose.position.y;
        T_odom_base(2,3) = odom->pose.pose.position.z;
        // chain: odom ← base_link ← velodyne
        lidarPoseMatrix = T_odom_base * T_base_velodyne_;
    }
    
    inline bool dynamicDetector::ensureStaticTfs() {
        if (staticTfsCached_) return true;
        bool ok = lookupTfMatrix("imu_link", "base_link", T_imu_base_) &&
                  lookupTfMatrix("base_link", this->tfLidarFrame_, T_base_velodyne_);
        if (ok) staticTfsCached_ = true;
        return ok;
    }

    inline onboardDetector::Point dynamicDetector::eigenToDBPoint(const Eigen::Vector3d& p){
        onboardDetector::Point pDB;
        pDB.x = p(0);
        pDB.y = p(1);
        pDB.z = p(2);
        pDB.clusterID = -1;
        return pDB;
    }

    inline Eigen::Vector3d dynamicDetector::dbPointToEigen(const onboardDetector::Point& pDB){
        Eigen::Vector3d p;
        p(0) = pDB.x;
        p(1) = pDB.y;
        p(2) = pDB.z;
        return p;
    }

    inline void dynamicDetector::eigenToDBPointVec(const std::vector<Eigen::Vector3d>& points, std::vector<onboardDetector::Point>& pointsDB, int size){
        for (int i=0; i<size; ++i){
            Eigen::Vector3d p = points[i];
            onboardDetector::Point pDB = this->eigenToDBPoint(p);
            pointsDB.push_back(pDB);
        }
    }
}

#endif
