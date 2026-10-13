/*
    FILE: dynamicDetector.cpp
    ---------------------------------
    function implementation of dynamic osbtacle detector
*/
#include <jo_detector/dynamicDetector.h>

namespace joDetector{
    // Ensure nextTrackId_ is initialized in the constructor
    dynamicDetector::dynamicDetector() : nextTrackId_(0) {
        this->ns_ = "jo_detector";
        this->hint_ = "[joDetector]";
    }

    dynamicDetector::dynamicDetector(const rclcpp::Node::SharedPtr& nh){
        this->ns_ = "jo_detector";
        this->hint_ = "[joDetector]";
        this->nh_ = nh;
        this->tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->nh_->get_clock());
        this->tfListener_ = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer_, this->nh_, false);
        this->initParam();
        this->registerPub();
        this->registerCallback();
    }

    void dynamicDetector::initDetector(const rclcpp::Node::SharedPtr& nh){
        this->nh_ = nh;
        this->tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->nh_->get_clock());
        this->tfListener_ = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer_, this->nh_, false);
        this->initParam();
        this->registerPub();
        this->registerCallback();
    }

    void dynamicDetector::initParam(){
                // Voxel size for filtering
                auto pname_voxel = [&](const std::string &p){ return this->ns_.empty() ? p : this->ns_ + "." + p; };
                if (!this->nh_->get_parameter(pname_voxel("voxel_size"), this->voxelSize_)) {
                    this->voxelSize_ = 0.1;
                    RCLCPP_WARN(this->nh_->get_logger(), "[dynamicDetector]: voxel_size not set, using default 0.1");
                } else {
                    RCLCPP_INFO(this->nh_->get_logger(), "[dynamicDetector]: voxel_size set to %f", this->voxelSize_);
                }
        // helper lambda to build parameter names
        auto pname = [&](const std::string &p){ return this->ns_.empty() ? p : this->ns_ + "." + p; };

        this->useTfPose_ = true;
        this->nh_->get_parameter(pname("use_tf_pose"), this->useTfPose_);

        if (!this->nh_->get_parameter(pname("tf_odom_frame"), this->tfOdomFrame_)) {
            this->tfOdomFrame_ = "odom";
            RCLCPP_WARN(this->nh_->get_logger(), "[dynamicDetector]: tf_odom_frame not set, using default '%s'", this->tfOdomFrame_.c_str());
        }

        if (!this->nh_->get_parameter(pname("tf_lidar_frame"), this->tfLidarFrame_)) {
            this->tfLidarFrame_ = "velodyne";
            RCLCPP_WARN(this->nh_->get_logger(), "[dynamicDetector]: tf_lidar_frame not set, using default '%s'", this->tfLidarFrame_.c_str());
        }

        if (!this->nh_->get_parameter(pname("tf_depth_frame"), this->tfDepthFrame_)) {
            this->tfDepthFrame_.clear();
            RCLCPP_ERROR(this->nh_->get_logger(), "[dynamicDetector]: tf_depth_frame not set. Configure it in YAML.");
        }

        if (!this->nh_->get_parameter(pname("tf_color_frame"), this->tfColorFrame_)) {
            this->tfColorFrame_.clear();
            RCLCPP_ERROR(this->nh_->get_logger(), "[dynamicDetector]: tf_color_frame not set. Configure it in YAML.");
        }

        RCLCPP_INFO(this->nh_->get_logger(),
                    "[dynamicDetector]: TF pose mode: %s (odom=%s, lidar=%s, depth=%s, color=%s)",
                    this->useTfPose_ ? "enabled" : "disabled",
                    this->tfOdomFrame_.c_str(),
                    this->tfLidarFrame_.c_str(),
                    this->tfDepthFrame_.c_str(),
                    this->tfColorFrame_.c_str());

        // localization mode
        if (!this->nh_->get_parameter(pname("localization_mode"), this->localizationMode_)){
            this->localizationMode_ = 0;
            std::cout << this->hint_ << ": No localization mode option. Use default: pose" << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Localization mode: pose (0)/odom (1). Your option: " << this->localizationMode_ << std::endl;
        }

        // depth topic name
        if (!this->nh_->get_parameter(pname("depth_image_topic"), this->depthTopicName_)){
            this->depthTopicName_ = "/camera/depth/image_raw";
            std::cout << this->hint_ << ": No depth image topic name. Use default: /camera/depth/image_raw" << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Depth topic: " << this->depthTopicName_ << std::endl;
        }

        // color topic name
        if (!this->nh_->get_parameter(pname("color_image_topic"), this->colorImgTopicName_)){
            this->colorImgTopicName_ = "/camera/color/image_raw";
            std::cout << this->hint_ << ": No color image topic name. Use default: /camera/color/image_raw" << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Color image topic: " << this->colorImgTopicName_ << std::endl;
        }

        // lidar topic name
        if (!this->nh_->get_parameter(pname("lidar_pointcloud_topic"), this->lidarTopicName_)){
            this->lidarTopicName_ = "/cloud_registered";
            std::cout << this->hint_ << ": No lidar pointcloud topic name. Use default: /cloud_registered" << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Lidar pointcloud topic: " << this->lidarTopicName_ << std::endl;
        }

        if (this->localizationMode_ == 0){
            if (!this->nh_->get_parameter(pname("pose_topic"), this->poseTopicName_)){
                this->poseTopicName_ = "/pose";
                std::cout << this->hint_ << ": No pose topic name. Use default: /pose" << std::endl;
            }
            else{
                std::cout << this->hint_ << ": Pose topic: " << this->poseTopicName_ << std::endl;
            }
        }

        if (this->localizationMode_ == 1){
            if (!this->nh_->get_parameter(pname("odom_topic"), this->odomTopicName_)){
                this->odomTopicName_ = "/odom";
                std::cout << this->hint_ << ": No odom topic name. Use default: /odom" << std::endl;
            }
            else{
                std::cout << this->hint_ << ": Odom topic: " << this->odomTopicName_ << std::endl;
            }
        }

        // depth intrinsics
        std::vector<double> depthIntrinsics (4);
        if (!this->nh_->get_parameter(pname("depth_intrinsics"), depthIntrinsics)){
            std::cout << this->hint_ << ": Please check camera intrinsics!" << std::endl;
            exit(0);
        }
        else{
            this->fx_ = depthIntrinsics[0];
            this->fy_ = depthIntrinsics[1];
            this->cx_ = depthIntrinsics[2];
            this->cy_ = depthIntrinsics[3];
            std::cout << this->hint_ << ": fx, fy, cx, cy: " << "["  << this->fx_ << ", " << this->fy_  << ", " << this->cx_ << ", "<< this->cy_ << "]" << std::endl;
        }

        // color intrinsics
        std::vector<double> colorIntrinsics (4);
        if (!this->nh_->get_parameter(pname("color_intrinsics"), colorIntrinsics)){
            std::cout << this->hint_ << ": Please check camera intrinsics!" << std::endl;
            exit(0);
        }
        else{
            this->fxC_ = colorIntrinsics[0];
            this->fyC_ = colorIntrinsics[1];
            this->cxC_ = colorIntrinsics[2];
            this->cyC_ = colorIntrinsics[3];
            std::cout << this->hint_ << ": fxC, fyC, cxC, cyC: " << "["  << this->fxC_ << ", " << this->fyC_  << ", " << this->cxC_ << ", "<< this->cyC_ << "]" << std::endl;
        }

        // depth scale factor
        if (!this->nh_->get_parameter(pname("depth_scale_factor"), this->depthScale_)){
            this->depthScale_ = 1000.0;
            std::cout << this->hint_ << ": No depth scale factor. Use default: 1000." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Depth scale factor: " << this->depthScale_ << std::endl;
        }

        // depth min value
        if (!this->nh_->get_parameter(pname("depth_min_value"), this->depthMinValue_)){
            this->depthMinValue_ = 0.2;
            std::cout << this->hint_ << ": No depth min value. Use default: 0.2 m." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Depth min value: " << this->depthMinValue_ << std::endl;
        }

        // depth max value
        if (!this->nh_->get_parameter(pname("depth_max_value"), this->depthMaxValue_)){
            this->depthMaxValue_ = 5.0;
            this->raycastMaxLength_ = 5.0;
            std::cout << this->hint_ << ": No depth max value. Use default: 5.0 m." << std::endl;
        }
        else{
            this->raycastMaxLength_ = this->depthMaxValue_;
            std::cout << this->hint_ << ": Depth max value: " << this->depthMaxValue_ << std::endl;
        }

        // depth filter margin
        if (!this->nh_->get_parameter(pname("depth_filter_margin"), this->depthFilterMargin_)){
            this->depthFilterMargin_ = 0;
            std::cout << this->hint_ << ": No depth filter margin. Use default: 0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Depth filter margin: " << this->depthFilterMargin_ << std::endl;
        }

        // depth skip pixel
        if (!this->nh_->get_parameter(pname("depth_skip_pixel"), this->skipPixel_)){
            this->skipPixel_ = 1;
            std::cout << this->hint_ << ": No depth skip pixel. Use default: 1." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Depth skip pixel: " << this->skipPixel_ << std::endl;
        }

        // ------------------------------------------------------------------------------------
        // depth image columns
        if (!this->nh_->get_parameter(pname("image_cols"), this->imgCols_)){
            this->imgCols_ = 640;
            std::cout << this->hint_ << ": No depth image columns. Use default: 640." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Depth image columns: " << this->imgCols_ << std::endl;
        }

        // depth rows
        if (!this->nh_->get_parameter(pname("image_rows"), this->imgRows_)){
            this->imgRows_ = 480;
            std::cout << this->hint_ << ": No depth image rows. Use default: 480." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Depth image rows: " << this->imgRows_ << std::endl;
        }
        this->projPoints_.resize(this->imgCols_ * this->imgRows_ / (this->skipPixel_ * this->skipPixel_));
        this->pointsDepth_.resize(this->imgCols_ * this->imgRows_ / (this->skipPixel_ * this->skipPixel_));
        // ------------------------------------------------------------------------------------


        RCLCPP_INFO(this->nh_->get_logger(), "[dynamicDetector]: Sensor extrinsic matrices are not loaded from YAML. Using TF-only pose computation.");

        // time step
        if (!this->nh_->get_parameter(pname("time_step"), this->dt_)){
            this->dt_ = 0.033;
            std::cout << this->hint_ << ": No time step parameter found. Use default: 0.033." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Time step for the system is set to: " << this->dt_ << std::endl;
        }  

        // ground height
        if (!this->nh_->get_parameter(pname("ground_height"), this->groundHeight_)){
            this->groundHeight_ = 0.1;
            std::cout << this->hint_ << ": No ground height parameter. Use default: 0.1m." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Ground height is set to: " << this->groundHeight_ << std::endl;
        }

        // roof height
        if (!this->nh_->get_parameter(pname("roof_height"), this->roofHeight_)){
            this->roofHeight_ = 2.0;
            std::cout << this->hint_ << ": No roof height parameter. Use default: 2.0m." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Roof height is set to: " << this->roofHeight_ << std::endl;
        }

        // min num of points for a voxel to be occupied in voxel filter
        if (!this->nh_->get_parameter(pname("voxel_occupied_thresh"), this->voxelOccThresh_)){
            this->voxelOccThresh_ = 10;
            std::cout << this->hint_ << ": No voxel_occupied_threshold. Use default: 10." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": min num of points for a voxel to be occupied in voxel filter is set to be: " << this->voxelOccThresh_ << std::endl;
        }

        // minimum number of points in each cluster
        if (!this->nh_->get_parameter(pname("dbscan_min_points_cluster"), this->dbMinPointsCluster_)){
            this->dbMinPointsCluster_ = 18;
            std::cout << this->hint_ << ": No DBSCAN minimum point in each cluster parameter. Use default: 18." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN Minimum point in each cluster is set to: " << this->dbMinPointsCluster_ << std::endl;
        }

        // ground/roof offset (roofHeight_ = groundHeight_ + offset)
        if (!this->nh_->get_parameter(pname("ground_roof_offset"), this->groundRoofOffset_)){
            this->groundRoofOffset_ = 5.0;
            std::cout << this->hint_ << ": No ground_roof_offset parameter. Use default: 5.0m." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Ground roof offset is set to: " << this->groundRoofOffset_ << std::endl;
        }

        // max zmin above ground for tracking (bboxes with zmin > groundHeight_ + this are skipped)
        if (!this->nh_->get_parameter(pname("track_max_zmin_above_ground"), this->trackMaxZminAboveGround_)){
            this->trackMaxZminAboveGround_ = 1.5;
            std::cout << this->hint_ << ": No track_max_zmin_above_ground parameter. Use default: 1.5m." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": track_max_zmin_above_ground is set to: " << this->trackMaxZminAboveGround_ << std::endl;
        }

        // initialize with safe defaults until wall_detector publishes ground estimation
        this->groundEstimated_ = false;

        // search range
        if (!this->nh_->get_parameter(pname("dbscan_search_range_epsilon"), this->dbEpsilon_)){
            this->dbEpsilon_ = 0.3;
            std::cout << this->hint_ << ": No DBSCAN epsilon parameter. Use default: 0.3." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN epsilon is set to: " << this->dbEpsilon_ << std::endl;
        }

        // lidar dbscan min points
        if (!this->nh_->get_parameter(pname("lidar_DBSCAN_min_points"), this->lidarDBMinPoints_)){
            this->lidarDBMinPoints_ = 10;
            std::cout << this->hint_ << ": No lidar DBSCAN minimum point in each cluster parameter. Use default: 10." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Lidar DBSCAN Minimum point in each cluster is set to: " << this->lidarDBMinPoints_ << std::endl;
        }

        // lidar dbscan search range
        if (!this->nh_->get_parameter(pname("lidar_DBSCAN_epsilon"), this->lidarDBEpsilon_)){
            this->lidarDBEpsilon_ = 0.2;
            std::cout << this->hint_ << ": No lidar DBSCAN epsilon parameter. Use default: 0.2." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Lidar DBSCAN epsilon is set to: " << this->lidarDBEpsilon_ << std::endl;
        }

        // lidar points downsample threshold
        if(!this->nh_->get_parameter(pname("downsample_threshold"), this->downSampleThresh_)){
            this->downSampleThresh_ = 4000;
            std::cout << this->hint_ << ": No downsample threshold parameter found. Use default: 4000." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Downsample threshold is set to: " << this->downSampleThresh_ << std::endl;
        }

        // gaussian downsample rate
        if (!this->nh_->get_parameter(pname("gaussian_downsample_rate"), this->gaussianDownSampleRate_)){
            this->gaussianDownSampleRate_ = 2;
            std::cout << this->hint_ << ": No gaussian downsample rate parameter found. Use default: 2." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Gaussian downsample rate is set to: " << this->gaussianDownSampleRate_ << std::endl;
        }

        // DBSCAN refinement enable
        if (!this->nh_->get_parameter(pname("dbscan_refinement_enable"), this->dbscanRefinementEnable_)){
            this->dbscanRefinementEnable_ = true;
            std::cout << this->hint_ << ": No DBSCAN refinement enable. Use default: true." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN refinement enable: " << this->dbscanRefinementEnable_ << std::endl;
        }

        // Visual DBSCAN refinement enable (separate from LiDAR)
        if (!this->nh_->get_parameter(pname("visual_dbscan_refinement_enable"), this->visualDbscanRefinementEnable_)){
            this->visualDbscanRefinementEnable_ = this->dbscanRefinementEnable_;
            std::cout << this->hint_ << ": No visual DBSCAN refinement enable. Use default from dbscan_refinement_enable: " << this->visualDbscanRefinementEnable_ << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Visual DBSCAN refinement enable: " << this->visualDbscanRefinementEnable_ << std::endl;
        }

        // DBSCAN refine max diagonal
        if (!this->nh_->get_parameter(pname("dbscan_refine_max_diagonal"), this->dbscanRefineMaxDiagonal_)){
            this->dbscanRefineMaxDiagonal_ = 2.0;
            std::cout << this->hint_ << ": No DBSCAN refine max diagonal. Use default: 2.0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN refine max diagonal: " << this->dbscanRefineMaxDiagonal_ << std::endl;
        }

        // DBSCAN refine min density
        if (!this->nh_->get_parameter(pname("dbscan_refine_min_density"), this->dbscanRefineMinDensity_)){
            this->dbscanRefineMinDensity_ = 80.0;
            std::cout << this->hint_ << ": No DBSCAN refine min density. Use default: 80.0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN refine min density: " << this->dbscanRefineMinDensity_ << std::endl;
        }

        // DBSCAN refine split min pts
        if (!this->nh_->get_parameter(pname("dbscan_refine_split_min_pts"), this->dbscanRefineSplitMinPts_)){
            this->dbscanRefineSplitMinPts_ = 8;
            std::cout << this->hint_ << ": No DBSCAN refine split min pts. Use default: 8." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN refine split min pts: " << this->dbscanRefineSplitMinPts_ << std::endl;
        }

        // DBSCAN refine split eps
        if (!this->nh_->get_parameter(pname("dbscan_refine_split_eps"), this->dbscanRefineSplitEps_)){
            this->dbscanRefineSplitEps_ = 0.08;
            std::cout << this->hint_ << ": No DBSCAN refine split eps. Use default: 0.08." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN refine split eps: " << this->dbscanRefineSplitEps_ << std::endl;
        }

        // DBSCAN refine min subcluster pts
        if (!this->nh_->get_parameter(pname("dbscan_refine_min_subcluster_pts"), this->dbscanRefineMinSubclusterPts_)){
            this->dbscanRefineMinSubclusterPts_ = 10;
            std::cout << this->hint_ << ": No DBSCAN refine min subcluster pts. Use default: 10." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN refine min subcluster pts: " << this->dbscanRefineMinSubclusterPts_ << std::endl;
        }

        // DBSCAN refine axis slice width
        if (!this->nh_->get_parameter(pname("dbscan_refine_axis_slice_width"), this->dbscanRefineAxisSliceWidth_)){
            this->dbscanRefineAxisSliceWidth_ = 0.40;
            std::cout << this->hint_ << ": No DBSCAN refine axis slice width. Use default: 0.40." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN refine axis slice width: " << this->dbscanRefineAxisSliceWidth_ << std::endl;
        }

        // DBSCAN refine max depth
        if (!this->nh_->get_parameter(pname("dbscan_refine_max_depth"), this->dbscanRefineMaxDepth_)){
            this->dbscanRefineMaxDepth_ = 2;
            std::cout << this->hint_ << ": No DBSCAN refine max depth. Use default: 2." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN refine max depth: " << this->dbscanRefineMaxDepth_ << std::endl;
        }

        // DBSCAN refine recursive
        if (!this->nh_->get_parameter(pname("dbscan_refine_recursive"), this->dbscanRefineRecursive_)){
            this->dbscanRefineRecursive_ = true;
            std::cout << this->hint_ << ": No DBSCAN refine recursive. Use default: true." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN refine recursive: " << this->dbscanRefineRecursive_ << std::endl;
        }

        // DBSCAN refine min box volume
        if (!this->nh_->get_parameter(pname("dbscan_refine_min_box_volume"), this->dbscanRefineMinBoxVolume_)){
            this->dbscanRefineMinBoxVolume_ = 0.0001;
            std::cout << this->hint_ << ": No DBSCAN refine min box volume. Use default: 0.0001." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN refine min box volume: " << this->dbscanRefineMinBoxVolume_ << std::endl;
        }

        // UV bounding box diagonal filter
        if (!this->nh_->get_parameter(pname("uv_box_max_diagonal"), this->uvBoxMaxDiagonal_)){
            this->uvBoxMaxDiagonal_ = 2.0; // meters, default
            std::cout << this->hint_ << ": No UV box max diagonal param found. Use default: 2.0." << std::endl;
        } else {
            std::cout << this->hint_ << ": UV box max diagonal is set to: " << this->uvBoxMaxDiagonal_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("box_max_height"), this->boxMaxHeight_) &&
            !this->nh_->get_parameter(pname("indoor_box_max_height"), this->boxMaxHeight_)){
            this->boxMaxHeight_ = 3.0;
            std::cout << this->hint_ << ": No box_max_height param found. Use default: 3.0 m." << std::endl;
        } else {
            std::cout << this->hint_ << ": Box max height is set to: " << this->boxMaxHeight_ << " m." << std::endl;
        }

        if (!this->nh_->get_parameter(pname("box_max_volume"), this->boxMaxVolume_) &&
            !this->nh_->get_parameter(pname("indoor_box_max_volume"), this->boxMaxVolume_)){
            this->boxMaxVolume_ = 3.0;
            std::cout << this->hint_ << ": No box_max_volume param found. Use default: 3.0 m^3." << std::endl;
        } else {
            std::cout << this->hint_ << ": Box max volume is set to: " << this->boxMaxVolume_ << " m^3." << std::endl;
        }

        // Visual IOU threshold
        if (!this->nh_->get_parameter(pname("visual_filtering_BBox_IOU_threshold"), this->visualboxIOUThresh_)){
            this->visualboxIOUThresh_ = 0.5;
            std::cout << this->hint_ << ": No threshold for boununding box IOU filtering parameter found. Use default: 0.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Threshold for boununding box IOU filtering is set to: " << this->visualboxIOUThresh_ << std::endl;
        }
        
        // Visual IOV threshold
        if (!this->nh_->get_parameter(pname("visual_filtering_BBox_IOV_threshold"), this->visualboxIOVThresh_)){
            this->visualboxIOVThresh_ = 0.5;
            std::cout << this->hint_ << ": No threshold for boununding box IOV filtering parameter found. Use default: 0.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Threshold for boununding box IOV filtering is set to: " << this->visualboxIOVThresh_ << std::endl;
        }

        // LiDAR Visual IOU threshold
        if (!this->nh_->get_parameter(pname("lidar_visual_filtering_BBox_IOU_threshold"), this->lidarVisualboxIOUThresh_)){
            this->lidarVisualboxIOUThresh_ = 0.5;
            std::cout << this->hint_ << ": No threshold for LiDAR bounding box IOU filtering parameter found. Use default: 0.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Threshold for LiDAR bounding box IOU filtering is set to: " << this->lidarVisualboxIOUThresh_ << std::endl;
        }
        
        // LiDAR Visual IOV threshold
        if (!this->nh_->get_parameter(pname("lidar_visual_filtering_BBox_IOV_threshold"), this->lidarVisualboxIOVThresh_)){
            this->lidarVisualboxIOVThresh_ = 0.5;
            std::cout << this->hint_ << ": No threshold for LiDAR bounding box IOV filtering parameter found. Use default: 0.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Threshold for LiDAR bounding box IOV filtering is set to: " << this->lidarVisualboxIOVThresh_ << std::endl;
        }

        // Visual Merging Flag
        if (!this->nh_->get_parameter(pname("visual_merging_flag"), this->visualmergingFlag_)){
            this->visualmergingFlag_ = "smaller";
            std::cout << this->hint_ << ": No visual merging flag parameter found. Use default: smaller." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Visual merging flag is set to: " << this->visualmergingFlag_ << std::endl;
        }

        // LiDAR Visual Merging Flag
        if (!this->nh_->get_parameter(pname("lidar_visual_merging_flag"), this->lidarVisualmergingFlag_)){
            this->lidarVisualmergingFlag_ = "smaller";
            std::cout << this->hint_ << ": No LiDAR visual merging flag parameter found. Use default: smaller." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": LiDAR visual merging flag is set to: " << this->lidarVisualmergingFlag_ << std::endl;
        }

        // LiDAR Visual Merger Leaf Only
        if (!this->nh_->get_parameter(pname("lidar_visual_merger_leaf_only"), this->lidarVisualMergerLeafOnly_)){
            this->lidarVisualMergerLeafOnly_ = false;
            std::cout << this->hint_ << ": No LiDAR visual merger leaf_only parameter found. Use default: false." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": LiDAR visual merger leaf_only is set to: " << this->lidarVisualMergerLeafOnly_ << std::endl;
        }

        // UV unmerged Flag
        if (!this->nh_->get_parameter(pname("uv_unmerged_flag"), this->uvUnmergedFlag_)){
            this->uvUnmergedFlag_ = false;
            std::cout << this->hint_ << ": No UV unmerged flag parameter found. Use default: false." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": UV unmerged flag is set to: " << (this->uvUnmergedFlag_ ? "true" : "false") << std::endl;
        }

        // DB unmerged Flag
        if (!this->nh_->get_parameter(pname("db_unmerged_flag"), this->dbUnmergedFlag_)){
            this->dbUnmergedFlag_ = false;
            std::cout << this->hint_ << ": No DB unmerged flag parameter found. Use default: false." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DB unmerged flag is set to: " << (this->dbUnmergedFlag_ ? "true" : "false") << std::endl;
        }

        // LiDAR unmerged Flag
        if (!this->nh_->get_parameter(pname("lidar_unmerged_flag"), this->lidarUnmergedFlag_)){
            this->lidarUnmergedFlag_ = true;
            std::cout << this->hint_ << ": No LiDAR unmerged flag parameter found. Use default: true." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": LiDAR unmerged flag is set to: " << (this->lidarUnmergedFlag_ ? "true" : "false") << std::endl;
        }

        // Same group IOV Threshold
        if (!this->nh_->get_parameter(pname("samegroupIOV_threshold"), this->samegroupIOVThresh_)){
            this->samegroupIOVThresh_ = 0.6;
            std::cout << this->hint_ << ": No same group IOV threshold parameter found. Use default: 0.6." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Same group IOV threshold is set to: " << this->samegroupIOVThresh_ << std::endl;
        }

        // Same group IOU Threshold
        if (!this->nh_->get_parameter(pname("samegroupIOU_threshold"), this->samegroupIOUThresh_)){
            this->samegroupIOUThresh_ = 0.3;
            std::cout << this->hint_ << ": No same group IOU threshold parameter found. Use default: 0.3." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Same group IOU threshold is set to: " << this->samegroupIOUThresh_ << std::endl;
        }

        // Final merge flag (extra mergeNestedGroup pass on filteredBBoxes before YOLO)
        if (!this->nh_->get_parameter(pname("final_merge_flag"), this->finalMergeFlag_)){
            this->finalMergeFlag_ = false;
            std::cout << this->hint_ << ": No final_merge_flag parameter found. Use default: false." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Final merge flag is set to: " << (this->finalMergeFlag_ ? "true" : "false") << std::endl;
        }

        // Visual unmerged Flag
        if (!this->nh_->get_parameter(pname("visual_unmerged_flag"), this->visualUnmergedFlag_)){
            this->visualUnmergedFlag_ = false;
            std::cout << this->hint_ << ": No visual unmerged flag parameter found. Use default: false." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Visual unmerged flag is set to: " << (this->visualUnmergedFlag_ ? "true" : "false") << std::endl;
        }

        // maximum match range
        if (!this->nh_->get_parameter(pname("max_match_range"), this->maxMatchRange_)){
            this->maxMatchRange_ = 0.5;
            std::cout << this->hint_ << ": No max match range parameter found. Use default: 0.5m." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Max match range is set to: " << this->maxMatchRange_  << "m." << std::endl;
        }

        // tracking history size
        if (!this->nh_->get_parameter(pname("history_size"), this->histSize_)){
            this->histSize_ = 5;
            std::cout << this->hint_ << ": No tracking history size parameter found. Use default: 5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": History for tracking is set to: " << this->histSize_ << std::endl;
        }

        // num of frames used in KF for observation
        if (!this->nh_->get_parameter(pname("kalman_filter_averaging_frames"), this->kfAvgFrames_)){
            this->kfAvgFrames_ = 10;
            std::cout << this->hint_ << ": No number of frames used in KF for observation parameter found. Use default: 10." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Number of frames used in KF for observation is set to: " << this->kfAvgFrames_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("prediction_visualization_steps"), this->predictionVisualizationSteps_)){
            this->predictionVisualizationSteps_ = 5;
            std::cout << this->hint_ << ": No prediction_visualization_steps parameter found. Use default: 5." << std::endl;
        }
        else if (this->predictionVisualizationSteps_ < 1){
            std::cout << this->hint_ << ": prediction_visualization_steps must be >= 1. Clamping to 1." << std::endl;
            this->predictionVisualizationSteps_ = 1;
        }
        else{
            std::cout << this->hint_ << ": Prediction visualization steps: " << this->predictionVisualizationSteps_ << std::endl;
        }

        // num of max missed frames for KF matching
        if (!this->nh_->get_parameter(pname("max_missed_frames"), this->maxMissedFrames_)){
            this->maxMissedFrames_ = 5;
            std::cout << this->hint_ << ": No number of max missed frames for KF matching parameter found. Use default: 5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Number of frames used in KF for observation is set to: " << this->kfAvgFrames_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("max_missed_frames_yolo"), this->maxMissedFramesYolo_)){
            this->maxMissedFramesYolo_ = 15;
            std::cout << this->hint_ << ": No max_missed_frames_yolo. Use default: 15." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": max_missed_frames_yolo set to: " << this->maxMissedFramesYolo_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("max_non_yolo_in_fov_frames"), this->maxNonYoloInFovFrames_)){
            this->maxNonYoloInFovFrames_ = 5;
            std::cout << this->hint_ << ": No max_non_yolo_in_fov_frames. Use default: 5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": max_non_yolo_in_fov_frames set to: " << this->maxNonYoloInFovFrames_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("max_yolo_base_mismatch_frames"), this->maxYoloBaseMismatchFrames_)){
            this->maxYoloBaseMismatchFrames_ = 5;
            std::cout << this->hint_ << ": No max_yolo_base_mismatch_frames. Use default: 5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": max_yolo_base_mismatch_frames set to: " << this->maxYoloBaseMismatchFrames_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("yolo_base_mismatch_thresh"), this->yoloBaseMismatchThresh_)){
            this->yoloBaseMismatchThresh_ = 0.50;
            std::cout << this->hint_ << ": No yolo_base_mismatch_thresh. Use default: 0.50." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": yolo_base_mismatch_thresh set to: " << this->yoloBaseMismatchThresh_ << std::endl;
        }

        // Kalman Filter V2 parameters (position-only observation model)
        std::vector<double> kfV2Params;
        if (!this->nh_->get_parameter(pname("kalman_filter_v2_param"), kfV2Params)){
            this->eP_v2_ = 0.25;
            this->eQPos_v2_ = 0.05;
            this->eQVel_v2_ = 0.15;
            this->eQAcc_v2_ = 0.10;
            this->eRPos_v2_ = 0.02;
            std::cout << this->hint_ << ": No KF v2 parameter found. Use defaults: [0.25, 0.05, 0.15, 0.10, 0.02]." << std::endl;
        }
        else{
            this->eP_v2_ = kfV2Params[0];
            this->eQPos_v2_ = kfV2Params[1];
            this->eQVel_v2_ = kfV2Params[2];
            this->eQAcc_v2_ = kfV2Params[3];
            this->eRPos_v2_ = kfV2Params[4];
            std::cout << this->hint_ << ": KF v2 parameter is set to: [";
            for (int i = 0; i < int(kfV2Params.size()); ++i){
                if (i != 0) std::cout << ", ";
                std::cout << kfV2Params[i];
            }
            std::cout << "]." << std::endl;
        }

        // minimum match score for matching
        if (!this->nh_->get_parameter(pname("min_match_score"), this->minMatchScore_)){
            this->minMatchScore_ = -2.5;
            std::cout << this->hint_ << ": No minimum match score parameter found. Use default: -2.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Minimum match score is set to: " << this->minMatchScore_ << std::endl;
        }

        // skip frame for classification
        if (!this->nh_->get_parameter(pname("frame_skip"), this->skipFrame_)){
            this->skipFrame_ = 5;
            std::cout << this->hint_ << ": No skip frame parameter found. Use default: 5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Frames skiped in classification when comparing two point cloud is set to: " << this->skipFrame_ << std::endl;
        }

        // velocity threshold for dynamic classification
        if (!this->nh_->get_parameter(pname("dynamic_velocity_threshold"), this->dynaVelThresh_)){
            this->dynaVelThresh_ = 0.35;
            std::cout << this->hint_ << ": No dynamic velocity threshold parameter found. Use default: 0.35." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Velocity threshold for dynamic classification is set to: " << this->dynaVelThresh_ << std::endl;
        }

        // voting threshold for dynamic classification
        if (!this->nh_->get_parameter(pname("dynamic_voting_threshold"), this->dynaVoteThresh_)){
            this->dynaVoteThresh_ = 0.7;
            std::cout << this->hint_ << ": No dynamic voting threshold parameter found. Use default: 0.7." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Voting threshold for dynamic classification is set to: " << this->dynaVoteThresh_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("dynamic_kf_vel_std_ratio"), this->dynaKfVelStdRatio_)){
            this->dynaKfVelStdRatio_ = 0.50;
            std::cout << this->hint_ << ": No dynamic_kf_vel_std_ratio parameter found. Use default: 0.50." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Kalman velocity std ratio for dynamic classification: " << this->dynaKfVelStdRatio_ << std::endl;
        }

        // frames to force dynamic
        if (!this->nh_->get_parameter(pname("frames_force_dynamic"), this->forceDynaFrames_)){
            this->forceDynaFrames_ = 20;
            std::cout << this->hint_ << ": No range of searching dynamic obstacles in box history found. Use default: 20." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Range of searching dynamic obstacles in box history is set to: " << this->forceDynaFrames_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("frames_force_dynamic_check_range"), this->forceDynaCheckRange_)){
            this->forceDynaCheckRange_ = 30;
            std::cout << this->hint_ << ": No threshold for forcing dynamic obstacles found. Use default: 30." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Threshold for forcing dynamic obstacles is set to: " << this->forceDynaCheckRange_ << std::endl;
        }

        // dynamic consistency check
        if (!this->nh_->get_parameter(pname("dynamic_consistency_threshold"), this->dynamicConsistThresh_)){
            this->dynamicConsistThresh_ = 3;
            std::cout << this->hint_ << ": No threshold for dynamic-consistency check found. Use default: 3." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Threshold for dynamic consistency check is set to: " << this->dynamicConsistThresh_ << std::endl;
        }

        if ( this->histSize_ < this->forceDynaCheckRange_+1){
            RCLCPP_ERROR(this->nh_->get_logger(), "history length is too short to perform force-dynamic");
        }

        // constrain target object size
        if (!this->nh_->get_parameter(pname("target_constrain_size"), this->constrainSize_)){
            this->constrainSize_ = false;
            std::cout << this->hint_ << ": No target object constrain size param found. Use default: false." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Target object constrain is set to: " << this->constrainSize_ << std::endl;
        }

        // target object  sizes
        std::vector<double> targetObjectSizeTemp;
        if (!this->nh_->get_parameter(pname("target_object_size"), targetObjectSizeTemp)){
            std::cout << this->hint_ << ": No target object size found. Do not apply target object size." << std::endl;
        }
        else{
            for (size_t i=0; i<targetObjectSizeTemp.size(); i+=3){
                Eigen::Vector3d targetSize (targetObjectSizeTemp[i+0], targetObjectSizeTemp[i+1], targetObjectSizeTemp[i+2]);
                this->targetObjectSize_.push_back(targetSize);
                std::cout << this->hint_ << ": target object size is set to: [" << targetObjectSizeTemp[i+0]  << ", " 
                << targetObjectSizeTemp[i+1] << ", " <<  targetObjectSizeTemp[i+2] << "]." << std::endl;
            }

        }

        // max object size
        std::vector<double> maxObjectSizeTemp;
        if(!this->nh_->get_parameter(pname("max_object_size"), maxObjectSizeTemp)){
            this->maxObjectSize_ = Eigen::Vector3d (2.0, 2.0, 2.0);
            std::cout << this->hint_ << ": No max object size threshold parameter found. Use default: [2.0, 2.0, 2.0]." << std::endl;
        }
        else{
            this->maxObjectSize_(0) = maxObjectSizeTemp[0];
            this->maxObjectSize_(1) = maxObjectSizeTemp[1];
            this->maxObjectSize_(2) = maxObjectSizeTemp[2];
            std::cout <<  this->hint_ << ": Max object size threshold is set to: [";
            for (size_t i = 0; i < maxObjectSizeTemp.size(); ++i) {
                std::cout << maxObjectSizeTemp[i];
                if (i != maxObjectSizeTemp.size()-1){
                    std::cout << ", ";
                }
            }
            std::cout << "]." << std::endl;
        }

        // Duplicate track distance threshold
        if (!this->nh_->get_parameter(pname("duplicate_track_dist_threshold"), this->duplicateTrackDistThresh_)){
            this->duplicateTrackDistThresh_ = 0.5;
            std::cout << this->hint_ << ": No duplicate track distance threshold parameter found.   Use default: 0.5." << std::endl;        
        }
        else{
            std::cout << this->hint_ << ": Duplicate track distance threshold is set to: " << this->duplicateTrackDistThresh_ << std::endl;
        }

        // Duplicate track IOU threshold
        if (!this->nh_->get_parameter(pname("duplicate_track_iou_threshold"), this->duplicateTrackIou2DThresh_)){
            this->duplicateTrackIou2DThresh_ = 0.3;
            std::cout << this->hint_ << ": No duplicate track IOU threshold parameter found. Use default: 0.3." << std::endl;        
        }
        else{
            std::cout << this->hint_ << ": Duplicate track IOU threshold is set to: " << this->duplicateTrackIou2DThresh_ << std::endl;
        }

        // Duplicate Size Rel Threshold
        if (!this->nh_->get_parameter(pname("duplicate_size_rel_threshold"), this->duplicateSizeRelThresh_)){
            this->duplicateSizeRelThresh_ = 0;
            std::cout << this->hint_ << ": No duplicate size relative threshold parameter found. Use default: 0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Duplicate size relative threshold is set to: " << this->duplicateSizeRelThresh_ << std::endl;
        }
        
        // Enable Tracking Debug Logs
        if (!this->nh_->get_parameter(pname("enable_tracking_debug_logs"), this->enableTrackingDebugLogs_)){
            this->enableTrackingDebugLogs_ = false;
            std::cout << this->hint_ << ": No enable tracking debug logs parameter found. Use default: false." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Enable tracking debug logs is set to: " << (this->enableTrackingDebugLogs_ ? "true" : "false") << std::endl;
        }

        // =========================
        // YOLO Dynamic Classification Parameters
        // =========================
        std::vector<std::string> yoloDynClassesTemp;
        if (this->nh_->get_parameter(pname("yolo_dynamic_classes"), yoloDynClassesTemp)){
            this->yoloDynamicClasses_ = yoloDynClassesTemp;
            std::cout << this->hint_ << ": YOLO dynamic classes: [";
            for (size_t i = 0; i < this->yoloDynamicClasses_.size(); ++i){
                std::cout << this->yoloDynamicClasses_[i];
                if (i != this->yoloDynamicClasses_.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }
        else{
            this->yoloDynamicClasses_ = {"person", "car", "bus", "truck", "motorbike", "bicycle", "dog", "cat", "horse", "cow", "sheep"};
            std::cout << this->hint_ << ": No yolo_dynamic_classes parameter. Using defaults: [person, car, bus, truck, motorbike, bicycle, dog, cat, horse, cow, sheep]." << std::endl;
        }

        if (!this->nh_->get_parameter(pname("yolo_point_fraction_threshold"), this->yoloPointFractionThresh_)){
            this->yoloPointFractionThresh_ = 0.15;
            std::cout << this->hint_ << ": No yolo_point_fraction_threshold. Use default: 0.15." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": YOLO point fraction threshold: " << this->yoloPointFractionThresh_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("yolo_sparse_large_box_volume_threshold"), this->yoloSparseLargeBoxVolumeThresh_)){
            this->yoloSparseLargeBoxVolumeThresh_ = 2.0;
            std::cout << this->hint_ << ": No yolo_sparse_large_box_volume_threshold. Use default: 2.0 m^3." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": YOLO sparse-large box volume threshold: " << this->yoloSparseLargeBoxVolumeThresh_ << " m^3" << std::endl;
        }

        if (!this->nh_->get_parameter(pname("yolo_sparse_large_min_points"), this->yoloSparseLargeMinPoints_)){
            this->yoloSparseLargeMinPoints_ = 20;
            std::cout << this->hint_ << ": No yolo_sparse_large_min_points. Use default: 20." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": YOLO sparse-large min points: " << this->yoloSparseLargeMinPoints_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("yolo_sparse_large_min_density"), this->yoloSparseLargeMinDensity_)){
            this->yoloSparseLargeMinDensity_ = 8.0;
            std::cout << this->hint_ << ": No yolo_sparse_large_min_density. Use default: 8.0 pts/m^3." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": YOLO sparse-large min density: " << this->yoloSparseLargeMinDensity_ << " pts/m^3" << std::endl;
        }

        if (!this->nh_->get_parameter(pname("yolo_max_height_ratio"), this->yoloMaxHeightRatio_)){
            this->yoloMaxHeightRatio_ = 2.5;
            std::cout << this->hint_ << ": No yolo_max_height_ratio. Use default: 2.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": YOLO max height ratio: " << this->yoloMaxHeightRatio_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("yolo_min_height_span"), this->yoloMinHeightSpan_)){
            this->yoloMinHeightSpan_ = 0.3;
            std::cout << this->hint_ << ": No yolo_min_height_span. Use default: 0.3 m." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": YOLO min height span: " << this->yoloMinHeightSpan_ << " m" << std::endl;
        }

        if (!this->nh_->get_parameter(pname("yolo_depth_tolerance"), this->yoloDepthTolerance_)){
            this->yoloDepthTolerance_ = 1.0;
            std::cout << this->hint_ << ": No yolo_depth_tolerance. Use default: 1.0 m." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": YOLO depth tolerance: " << this->yoloDepthTolerance_ << " m" << std::endl;
        }

        if (!this->nh_->get_parameter(pname("yolo_outdoor_depth_tolerance"), this->yoloOutdoorDepthTolerance_)){
            this->yoloOutdoorDepthTolerance_ = 1.5;
            std::cout << this->hint_ << ": No yolo_outdoor_depth_tolerance. Use default: 1.5 m." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": YOLO outdoor depth tolerance: " << this->yoloOutdoorDepthTolerance_ << " m" << std::endl;
        }

        if (!this->nh_->get_parameter(pname("yolo_height_correction_threshold"), this->yoloHeightCorrectionThreshold_)){
            this->yoloHeightCorrectionThreshold_ = 1.0;
        }
        if (!this->nh_->get_parameter(pname("yolo_2d_iou_threshold"), this->yolo2dIouThreshold_)){
            this->yolo2dIouThreshold_ = 0.30;
        }
        if (!this->nh_->get_parameter(pname("yolo_x_y_resize"), this->yoloXYResize_)){
            this->yoloXYResize_ = false;
            std::cout << this->hint_ << ": No yolo_x_y_resize. Use default: false." << std::endl;
        } else {
            std::cout << this->hint_ << ": yolo_x_y_resize: " << (this->yoloXYResize_ ? "true" : "false") << std::endl;
        }
        if (!this->nh_->get_parameter(pname("is_indoor"), this->isIndoor_)){
            this->isIndoor_ = false;
            std::cout << this->hint_ << ": No is_indoor parameter. Use default: false (outdoor)." << std::endl;
        } else {
            std::cout << this->hint_ << ": Environment mode: " << (this->isIndoor_ ? "indoor" : "outdoor") << std::endl;
        }

        // =========================
        // Sensor and LiDAR Range Parameters
        // =========================
        std::vector<double> sensorRangeTemp;
        if (this->nh_->get_parameter(pname("local_sensor_range"), sensorRangeTemp) && sensorRangeTemp.size() == 3){
            this->localSensorRange_ = Eigen::Vector3d(sensorRangeTemp[0], sensorRangeTemp[1], sensorRangeTemp[2]);
            std::cout << this->hint_ << ": Local sensor range: [" << sensorRangeTemp[0] << ", " << sensorRangeTemp[1] << ", " << sensorRangeTemp[2] << "]" << std::endl;
        }
        else{
            this->localSensorRange_ = Eigen::Vector3d(5.0, 5.0, 5.0);
            std::cout << this->hint_ << ": No local_sensor_range. Use default: [5.0, 5.0, 5.0]." << std::endl;
        }

        std::vector<double> lidarRangeTemp;
        if (this->nh_->get_parameter(pname("local_lidar_range"), lidarRangeTemp) && lidarRangeTemp.size() == 3){
            this->localLidarRange_ = Eigen::Vector3d(lidarRangeTemp[0], lidarRangeTemp[1], lidarRangeTemp[2]);
            std::cout << this->hint_ << ": Local LiDAR range: [" << lidarRangeTemp[0] << ", " << lidarRangeTemp[1] << ", " << lidarRangeTemp[2] << "]" << std::endl;
        }
        else{
            this->localLidarRange_ = Eigen::Vector3d(10.0, 10.0, 5.0);
            std::cout << this->hint_ << ": No local_lidar_range. Use default: [10.0, 10.0, 5.0]." << std::endl;
        }

        // =========================
        // Association Robustness Parameters
        // =========================
        if (!this->nh_->get_parameter(pname("max_match_speed"), this->maxMatchSpeed_)){
            this->maxMatchSpeed_ = 8.0;
            std::cout << this->hint_ << ": No max_match_speed. Use default: 8.0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Max match speed: " << this->maxMatchSpeed_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("match_pos_score_weight"), this->matchPosScoreWeight_)){
            this->matchPosScoreWeight_ = 1.0;
            std::cout << this->hint_ << ": No match_pos_score_weight. Use default: 1.0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Match pos score weight: " << this->matchPosScoreWeight_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("match_size_score_weight"), this->matchSizeScoreWeight_)){
            this->matchSizeScoreWeight_ = 0.35;
            std::cout << this->hint_ << ": No match_size_score_weight. Use default: 0.35." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Match size score weight: " << this->matchSizeScoreWeight_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("match_iou2d_score_weight"), this->matchIou2DScoreWeight_)){
            this->matchIou2DScoreWeight_ = 0.15;
            std::cout << this->hint_ << ": No match_iou2d_score_weight. Use default: 0.15." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Match IoU2D score weight: " << this->matchIou2DScoreWeight_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("max_relative_size_diff_match"), this->maxRelativeSizeDiffMatch_)){
            this->maxRelativeSizeDiffMatch_ = 0.60;
            std::cout << this->hint_ << ": No max_relative_size_diff_match. Use default: 0.60." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Max relative size diff match: " << this->maxRelativeSizeDiffMatch_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("match_velocity_direction_score_weight"), this->matchVelocityDirectionScoreWeight_)){
            this->matchVelocityDirectionScoreWeight_ = 0.12;
            std::cout << this->hint_ << ": No match_velocity_direction_score_weight. Use default: 0.12." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Match velocity direction score weight: " << this->matchVelocityDirectionScoreWeight_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("match_yolo_class_consistency_weight"), this->matchYoloClassConsistencyWeight_)){
            this->matchYoloClassConsistencyWeight_ = 0.50;
            std::cout << this->hint_ << ": No match_yolo_class_consistency_weight. Use default: 0.50." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Match YOLO class consistency weight: " << this->matchYoloClassConsistencyWeight_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("match_prev_obs_pos_score_weight"), this->matchPrevObsPosScoreWeight_)){
            this->matchPrevObsPosScoreWeight_ = 0.25;
            std::cout << this->hint_ << ": No match_prev_obs_pos_score_weight. Use default: 0.25." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Match prev obs pos score weight: " << this->matchPrevObsPosScoreWeight_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("match_prev_obs_iou2d_score_weight"), this->matchPrevObsIou2DScoreWeight_)){
            this->matchPrevObsIou2DScoreWeight_ = 0.10;
            std::cout << this->hint_ << ": No match_prev_obs_iou2d_score_weight. Use default: 0.10." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Match prev obs IoU2D score weight: " << this->matchPrevObsIou2DScoreWeight_ << std::endl;
        }

        // =========================
        // Track Confirmation / Natural Motion Parameters
        // =========================
        if (!this->nh_->get_parameter(pname("min_confirm_hits"), this->minConfirmHits_)){
            this->minConfirmHits_ = 3;
            std::cout << this->hint_ << ": No min_confirm_hits. Use default: 3." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Min confirm hits: " << this->minConfirmHits_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("min_natural_motion_dist"), this->minNaturalMotionDist_)){
            this->minNaturalMotionDist_ = 0.08;
            std::cout << this->hint_ << ": No min_natural_motion_dist. Use default: 0.08." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Min natural motion dist: " << this->minNaturalMotionDist_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("max_natural_motion_dist"), this->maxNaturalMotionDist_)){
            this->maxNaturalMotionDist_ = 1.50;
            std::cout << this->hint_ << ": No max_natural_motion_dist. Use default: 1.50." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Max natural motion dist: " << this->maxNaturalMotionDist_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("max_natural_innovation"), this->maxNaturalInnovation_)){
            this->maxNaturalInnovation_ = 0.60;
            std::cout << this->hint_ << ": No max_natural_innovation. Use default: 0.60." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Max natural innovation: " << this->maxNaturalInnovation_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("max_velocity_direction_error_confirm"), this->maxVelocityDirectionErrorConfirm_)){
            this->maxVelocityDirectionErrorConfirm_ = 1.20;
            std::cout << this->hint_ << ": No max_velocity_direction_error_confirm. Use default: 1.20." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Max velocity direction error confirm: " << this->maxVelocityDirectionErrorConfirm_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("max_velocity_direction_error_tracked"), this->maxVelocityDirectionErrorTracked_)){
            this->maxVelocityDirectionErrorTracked_ = 2.50;
            std::cout << this->hint_ << ": No max_velocity_direction_error_tracked. Use default: 2.50." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Max velocity direction error tracked: " << this->maxVelocityDirectionErrorTracked_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("stationary_speed_thresh"), this->stationarySpeedThresh_)){
            this->stationarySpeedThresh_ = 0.10;
            std::cout << this->hint_ << ": No stationary_speed_thresh. Use default: 0.10." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Stationary speed thresh: " << this->stationarySpeedThresh_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("track_steady_objects"), this->trackSteadyObjects_)){
            this->trackSteadyObjects_ = false;
            std::cout << this->hint_ << ": No track_steady_objects. Use default: false." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Track steady objects: " << (this->trackSteadyObjects_ ? "true" : "false") << std::endl;
        }

        if (!this->nh_->get_parameter(pname("confirmed_track_assoc_bonus"), this->confirmedTrackAssocBonus_)){
            this->confirmedTrackAssocBonus_ = 0.20;
            std::cout << this->hint_ << ": No confirmed_track_assoc_bonus. Use default: 0.20." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Confirmed track assoc bonus: " << this->confirmedTrackAssocBonus_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("yolo_track_assoc_bonus"), this->yoloTrackAssocBonus_)){
            this->yoloTrackAssocBonus_ = 0.25;
            std::cout << this->hint_ << ": No yolo_track_assoc_bonus. Use default: 0.25." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": YOLO track assoc bonus: " << this->yoloTrackAssocBonus_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("max_natural_innovation_confirmed"), this->maxNaturalInnovationConfirmed_)){
            this->maxNaturalInnovationConfirmed_ = 1.00;
            std::cout << this->hint_ << ": No max_natural_innovation_confirmed. Use default: 1.00." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Max natural innovation confirmed: " << this->maxNaturalInnovationConfirmed_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("max_natural_innovation_dynamic"), this->maxNaturalInnovationDynamic_)){
            this->maxNaturalInnovationDynamic_ = 1.20;
            std::cout << this->hint_ << ": No max_natural_innovation_dynamic. Use default: 1.20." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Max natural innovation dynamic: " << this->maxNaturalInnovationDynamic_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("max_velocity_direction_error_confirm_dynamic"), this->maxVelocityDirectionErrorConfirmDynamic_)){
            this->maxVelocityDirectionErrorConfirmDynamic_ = 2.20;
            std::cout << this->hint_ << ": No max_velocity_direction_error_confirm_dynamic. Use default: 2.20." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Max velocity direction error confirm dynamic: " << this->maxVelocityDirectionErrorConfirmDynamic_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("max_velocity_direction_error_tracked_dynamic"), this->maxVelocityDirectionErrorTrackedDynamic_)){
            this->maxVelocityDirectionErrorTrackedDynamic_ = 4.00;
            std::cout << this->hint_ << ": No max_velocity_direction_error_tracked_dynamic. Use default: 4.00." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Max velocity direction error tracked dynamic: " << this->maxVelocityDirectionErrorTrackedDynamic_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("min_match_score_confirmed"), this->minMatchScoreConfirmed_)){
            this->minMatchScoreConfirmed_ = -1.20;
            std::cout << this->hint_ << ": No min_match_score_confirmed. Use default: -1.20." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Min match score confirmed: " << this->minMatchScoreConfirmed_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("min_match_score_dynamic"), this->minMatchScoreDynamic_)){
            this->minMatchScoreDynamic_ = -1.50;
            std::cout << this->hint_ << ": No min_match_score_dynamic. Use default: -1.50." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Min match score dynamic: " << this->minMatchScoreDynamic_ << std::endl;
        }

        // =========================
        // Outside-FOV Association Parameters
        // =========================
        if (!this->nh_->get_parameter(pname("min_confirm_hits_outside_fov"), this->minConfirmHitsOutsideFov_)){
            this->minConfirmHitsOutsideFov_ = 3;
        }
        if (!this->nh_->get_parameter(pname("min_outside_fov_obs_speed"), this->minOutsideFovObsSpeed_)){
            this->minOutsideFovObsSpeed_ = 0.10;
        }
        if (!this->nh_->get_parameter(pname("max_outside_fov_obs_speed"), this->maxOutsideFovObsSpeed_)){
            this->maxOutsideFovObsSpeed_ = 3.00;
        }
        if (!this->nh_->get_parameter(pname("max_natural_innovation_outside_fov"), this->maxNaturalInnovationOutsideFov_)){
            this->maxNaturalInnovationOutsideFov_ = 1.00;
        }
        if (!this->nh_->get_parameter(pname("max_velocity_direction_error_confirm_outside_fov"), this->maxVelocityDirectionErrorConfirmOutsideFov_)){
            this->maxVelocityDirectionErrorConfirmOutsideFov_ = 3.00;
        }
        if (!this->nh_->get_parameter(pname("outside_fov_obs_vel_window"), this->outsideFovObsVelWindow_)){
            this->outsideFovObsVelWindow_ = 3;
        }
        if (!this->nh_->get_parameter(pname("min_outside_fov_avg_obs_speed"), this->minOutsideFovAvgObsSpeed_)){
            this->minOutsideFovAvgObsSpeed_ = 0.10;
        }
        if (!this->nh_->get_parameter(pname("max_outside_fov_avg_obs_speed"), this->maxOutsideFovAvgObsSpeed_)){
            this->maxOutsideFovAvgObsSpeed_ = 30.00;
        }
        if (!this->nh_->get_parameter(pname("max_match_range_outside_fov"), this->maxMatchRangeOutsideFov_)){
            this->maxMatchRangeOutsideFov_ = 1.20;
        }
        if (!this->nh_->get_parameter(pname("max_match_speed_outside_fov"), this->maxMatchSpeedOutsideFov_)){
            this->maxMatchSpeedOutsideFov_ = 4.00;
        }
        if (!this->nh_->get_parameter(pname("max_relative_size_diff_outside_fov"), this->maxRelativeSizeDiffOutsideFov_)){
            this->maxRelativeSizeDiffOutsideFov_ = 0.85;
        }
        if (!this->nh_->get_parameter(pname("outside_fov_confirmed_assoc_bonus"), this->outsideFovConfirmedAssocBonus_)){
            this->outsideFovConfirmedAssocBonus_ = 0.75;
        }
        if (!this->nh_->get_parameter(pname("outside_fov_tentative_assoc_bonus"), this->outsideFovTentativeAssocBonus_)){
            this->outsideFovTentativeAssocBonus_ = 0.35;
        }
        if (!this->nh_->get_parameter(pname("outside_fov_turn_max_pos_dist"), this->outsideFovTurnMaxPosDist_)){
            this->outsideFovTurnMaxPosDist_ = 0.80;
        }
        if (!this->nh_->get_parameter(pname("outside_fov_turn_max_speed"), this->outsideFovTurnMaxSpeed_)){
            this->outsideFovTurnMaxSpeed_ = 3.50;
        }
        if (!this->nh_->get_parameter(pname("outside_fov_turn_max_size_diff"), this->outsideFovTurnMaxSizeDiff_)){
            this->outsideFovTurnMaxSizeDiff_ = 0.90;
        }
        if (!this->nh_->get_parameter(pname("outside_fov_turn_min_obs_speed"), this->outsideFovTurnMinObsSpeed_)){
            this->outsideFovTurnMinObsSpeed_ = 0.15;
        }
        if (!this->nh_->get_parameter(pname("outside_fov_turn_assoc_bonus"), this->outsideFovTurnAssocBonus_)){
            this->outsideFovTurnAssocBonus_ = 0.60;
        }
        if (!this->nh_->get_parameter(pname("outside_fov_turn_min_track_age"), this->outsideFovTurnMinTrackAge_)){
            this->outsideFovTurnMinTrackAge_ = 5;
        }
        if (!this->nh_->get_parameter(pname("outside_fov_class_window"), this->outsideFovClassWindow_)){
            this->outsideFovClassWindow_ = 5;
        }
        if (!this->nh_->get_parameter(pname("outside_fov_class_min_track_age"), this->outsideFovClassMinTrackAge_)){
            this->outsideFovClassMinTrackAge_ = 5;
        }
        if (!this->nh_->get_parameter(pname("outside_fov_class_min_net_disp"), this->outsideFovClassMinNetDisp_)){
            this->outsideFovClassMinNetDisp_ = 0.08;
        }
        if (!this->nh_->get_parameter(pname("outside_fov_class_min_net_speed"), this->outsideFovClassMinNetSpeed_)){
            this->outsideFovClassMinNetSpeed_ = 0.12;
        }
        if (!this->nh_->get_parameter(pname("outside_fov_class_min_straightness"), this->outsideFovClassMinStraightness_)){
            this->outsideFovClassMinStraightness_ = 0.45;
        }
        if (!this->nh_->get_parameter(pname("outside_fov_class_max_step_speed"), this->outsideFovClassMaxStepSpeed_)){
            this->outsideFovClassMaxStepSpeed_ = 20.00;
        }

    }

    bool dynamicDetector::lookupTfMatrix(const std::string& targetFrame, const std::string& sourceFrame, Eigen::Matrix4d& transformMatrix){
        if (!this->tfBuffer_) {
            return false;
        }

        try {
            geometry_msgs::msg::TransformStamped tfStamped =
                this->tfBuffer_->lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero);

            const auto& t = tfStamped.transform.translation;
            const auto& q = tfStamped.transform.rotation;

            Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
            Eigen::Matrix3d rot = quat.toRotationMatrix();

            transformMatrix.setIdentity();
            transformMatrix.block<3, 3>(0, 0) = rot;
            transformMatrix(0, 3) = t.x;
            transformMatrix(1, 3) = t.y;
            transformMatrix(2, 3) = t.z;


            return true;
        }
        catch (const tf2::TransformException&) {
            return false;
        }
    }

    void dynamicDetector::registerPub(){
        // uv detector depth map pub
        this->uvDepthMapPub_ = this->nh_->create_publisher<sensor_msgs::msg::Image>(
            this->ns_.empty() ? "/detected_depth_map" : this->ns_ + "/detected_depth_map", 10);

        // uv detector u depth map pub
        this->uDepthMapPub_ = this->nh_->create_publisher<sensor_msgs::msg::Image>(
            this->ns_.empty() ? "/detected_u_depth_map" : this->ns_ + "/detected_u_depth_map", 10);

        // uv detector bird view pub
        this->uvBirdViewPub_ = this->nh_->create_publisher<sensor_msgs::msg::Image>(
            this->ns_.empty() ? "/u_depth_bird_view" : this->ns_ + "/u_depth_bird_view", 10);

        // color 2D bounding boxes pub
        this->detectedColorImgPub_ = this->nh_->create_publisher<sensor_msgs::msg::Image>(
            this->ns_.empty() ? "/detected_color_image" : this->ns_ + "/detected_color_image", 10);

        // uv detector bounding box pub
        this->uvBBoxesPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/uv_bboxes" : this->ns_ + "/uv_bboxes", 10);

        // uv detector filtered bounding box pub
        this->uvBBoxesFilteredPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/uv_bboxes_filtered" : this->ns_ + "/uv_bboxes_filtered", 10);

        // DBSCAN bounding box pub
        this->dbBBoxesPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/dbscan_bboxes" : this->ns_ + "/dbscan_bboxes", 10);
        
        //DBSCAN filtered bounding box pub
        this->dbBBoxesFilteredPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/dbscan_bboxes_filtered" : this->ns_ + "/dbscan_bboxes_filtered", 10);

        // visual bboxes pub
        this->visualBBoxesPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/visual_bboxes" : this->ns_ + "/visual_bboxes", 10);

        // lidar bbox pub
        this->lidarBBoxesPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/lidar_bboxes" : this->ns_ + "/lidar_bboxes", 10);

        // lidar filtered bounding box pub
        this->lidarBBoxesFilteredPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/lidar_bboxes_filtered" : this->ns_ + "/lidar_bboxes_filtered", 10);

        // filtered bounding box before YOLO pub
        this->filteredBBoxesBeforeYoloPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/filtered_before_yolo_bboxes" : this->ns_ + "/filtered_before_yolo_bboxes", 10);

        // filtered bounding box pub
        this->filteredBBoxesPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/filtered_bboxes" : this->ns_ + "/filtered_bboxes", 10);

        // tracked bounding box pub (visualization only)
        this->predictedBBoxesActivePub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/predicted_bboxes_active" : this->ns_ + "/predicted_bboxes_active", 10);
        this->predictedBBoxesMissedPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/predicted_bboxes_missed" : this->ns_ + "/predicted_bboxes_missed", 10);
        this->predictedBBoxesUnconfirmedPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/predicted_bboxes_unconfirmed" : this->ns_ + "/predicted_bboxes_unconfirmed", 10);

        this->trackedBBoxesPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/tracked_bboxes" : this->ns_ + "/tracked_bboxes", 10);

        // tracked obstacles pub (machine-to-machine interface for Nav2)
        this->trackedObstaclesPub_ = this->nh_->create_publisher<jo_msgs::msg::ObstacleArray>(
            this->ns_.empty() ? "/tracked_dynamic_obstacles" : this->ns_ + "/tracked_dynamic_obstacles", 10);
        this->trackedObstaclesBBoxesPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/tracked_dynamic_obstacles_bboxes" : this->ns_ + "/tracked_dynamic_obstacles_bboxes", 10);

        // dynamic bounding box pub
        this->dynamicBBoxesPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/dynamic_bboxes" : this->ns_ + "/dynamic_bboxes", 10);
        this->potentiallyDynamicBBoxesPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/potentially_dynamic_bboxes" : this->ns_ + "/potentially_dynamic_bboxes", 10);

        // filtered depth pointcloud pub
        this->filteredDepthPointsPub_ = this->nh_->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_.empty() ? "/filtered_depth_cloud" : this->ns_ + "/filtered_depth_cloud", 10);

        // lidar cluster pub
        this->lidarClustersPub_ = this->nh_->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_.empty() ? "/lidar_clusters" : this->ns_ + "/lidar_clusters", 10);

        // filtered pointcloud pub
        this->filteredPointsPub_ = this->nh_->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_.empty() ? "/filtered_point_cloud" : this->ns_ + "/filtered_point_cloud", 10);

        // dynamic pointcloud pub
        this->dynamicPointsPub_ = this->nh_->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_.empty() ? "/dynamic_point_cloud" : this->ns_ + "/dynamic_point_cloud", 10);

        // raw dynamic pointcloud pub
        this->rawDynamicPointsPub_ = this->nh_->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_.empty() ? "/raw_dynamic_point_cloud" : this->ns_ + "/raw_dynamic_point_cloud", 10);

        // downsample points visualization pub
        this->downSamplePointsPub_ = this->nh_->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_.empty() ? "/downsampled_point_cloud" : this->ns_ + "/downsampled_point_cloud", 10);

        // stable public-API republish of the same cloud, for other packages
        // (jo_navigation costmap, wall_detector, ...) to consume instead of
        // re-filtering raw /velodyne_points themselves. See processedLidarPub_
        // declaration in dynamicDetector.h.
        this->processedLidarPub_ = this->nh_->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_.empty() ? "/jo_detector/lidar/processed" : this->ns_ + "/jo_detector/lidar/processed", 10);

        // raw LiDAR points visualization pub
        this->rawLidarPointsPub_ = this->nh_->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_.empty() ? "/raw_lidar_point_cloud" : this->ns_ + "/raw_lidar_point_cloud", 10);

        // history trajectory pub
        this->historyTrajPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(this->ns_.empty() ? "/history_trajectories" : this->ns_ + "/history_trajectories", 10);

        // velocity visualization pub
        this->velVisPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(this->ns_.empty() ? "/velocity_visualizaton" : this->ns_ + "/velocity_visualizaton", 10);
        
        // filtered lidar points in velodyne frame pub
        this->filteredLidarVelodynePub_ = this->nh_->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_.empty() ? "/filtered_lidar_points_velodyne_frame" : this->ns_ + "/filtered_lidar_points_velodyne_frame", 10);

        // YOLO foreground points pub
        this->yoloPointsPub_ = this->nh_->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_.empty() ? "/yolo_points" : this->ns_ + "/yolo_points", 10);
    }   

    void dynamicDetector::registerCallback(){
        // depth pose callback. "compressedDepth" transport: the
        // compressed_depth_image_transport plugin decompresses transparently
        // on receipt, so depthPoseCB/depthOdomCB below still get a normal,
        // uncompressed sensor_msgs::msg::Image — nothing else changes.
        this->depthSub_ = std::make_shared<image_transport::SubscriberFilter>();
        this->depthSub_->subscribe(this->nh_.get(), this->depthTopicName_, "compressedDepth", rmw_qos_profile_sensor_data);
        this->lidarCloudSub_.reset(new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(this->nh_, this->lidarTopicName_));
        if (this->localizationMode_ == 0){
            this->poseSub_.reset(new message_filters::Subscriber<geometry_msgs::msg::PoseStamped>(this->nh_, this->poseTopicName_));
            // ApproximateTime with large queue for rosbag playback
            this->depthPoseSync_.reset(new message_filters::Synchronizer<depthPoseSync>(depthPoseSync(1000), *this->depthSub_, *this->poseSub_));
            this->depthPoseSync_->registerCallback(std::bind(&dynamicDetector::depthPoseCB, this, std::placeholders::_1, std::placeholders::_2));
            
            this->lidarPoseSync_.reset(new message_filters::Synchronizer<lidarPoseSync>(lidarPoseSync(1000), *this->lidarCloudSub_, *this->poseSub_));
            this->lidarPoseSync_->registerCallback(std::bind(&dynamicDetector::lidarPoseCB, this, std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(this->nh_->get_logger(), "[dynamicDetector]: POSE mode - ApproximateTime sync (queue=1000)");
        }
        else if (this->localizationMode_ == 1){
            this->odomSub_.reset(new message_filters::Subscriber<nav_msgs::msg::Odometry>(this->nh_, this->odomTopicName_));
            // ApproximateTime with large queue for rosbag playback
            this->depthOdomSync_.reset(new message_filters::Synchronizer<depthOdomSync>(depthOdomSync(1000), *this->depthSub_, *this->odomSub_));
            this->depthOdomSync_->registerCallback(std::bind(&dynamicDetector::depthOdomCB, this, std::placeholders::_1, std::placeholders::_2));
            
            this->lidarOdomSync_.reset(new message_filters::Synchronizer<lidarOdomSync>(lidarOdomSync(1000), *this->lidarCloudSub_, *this->odomSub_));
            this->lidarOdomSync_->registerCallback(std::bind(&dynamicDetector::lidarOdomCB, this, std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(this->nh_->get_logger(), "[dynamicDetector]: ODOMETRY mode - ApproximateTime sync (queue=1000)");
        }
        else{
            RCLCPP_ERROR(this->nh_->get_logger(), "[dynamicDetector]: Invalid localization mode!");
            exit(0);
        }

        // color image subscriber. "compressed" transport: the
        // compressed_image_transport plugin decompresses (JPEG/PNG)
        // transparently on receipt, so colorImgCB below still gets a normal,
        // uncompressed sensor_msgs::msg::Image.
        this->colorImgSub_ = image_transport::create_subscription(
            this->nh_.get(), this->colorImgTopicName_,
            std::bind(&dynamicDetector::colorImgCB, this, std::placeholders::_1),
            "compressed", rmw_qos_profile_sensor_data);

        // yolo detection results subscriber
        this->yoloDetectionSub_ = this->nh_->create_subscription<vision_msgs::msg::Detection2DArray>(
            "yolo_detector/detected_bounding_boxes", 10,
            std::bind(&dynamicDetector::yoloDetectionCB, this, std::placeholders::_1));

        // wall detector results subscribers
        this->groundHeightSub_ = this->nh_->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/wall_detector/ground_height", 10,
            std::bind(&dynamicDetector::groundHeightCB, this, std::placeholders::_1));
        this->wallMarkersSub_ = this->nh_->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/wall_detector/wall_markers", 10,
            std::bind(&dynamicDetector::wallMarkersCB, this, std::placeholders::_1));

        // main pipeline timer: detect (parallel) → tracking → classification
        this->mainTimer_ = this->nh_->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(this->dt_ * 1000.0)),
            std::bind(&dynamicDetector::mainCB, this));

        // visualization runs independently so it never blocks the detection pipeline
        this->visTimer_ = this->nh_->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(this->dt_ * 1000.0)),
            std::bind(&dynamicDetector::visCB, this));

        // get dynamic obstacle service
        this->getDynamicObstacleServer_ = this->nh_->create_service<jo_detector::srv::GetDynamicObstacles>(
            "jo_detector/get_dynamic_obstacles",
            [this](const std::shared_ptr<jo_detector::srv::GetDynamicObstacles::Request> req,
                   std::shared_ptr<jo_detector::srv::GetDynamicObstacles::Response> res) -> void {
                this->getDynamicObstacles(req, res);
            });
    }

    bool dynamicDetector::getDynamicObstacles(const std::shared_ptr<jo_detector::srv::GetDynamicObstacles::Request> req,
                                              std::shared_ptr<jo_detector::srv::GetDynamicObstacles::Response> res) {
        // Get the current robot position
        Eigen::Vector3d currPos = Eigen::Vector3d (req->current_position.x, req->current_position.y, req->current_position.z);

        // Vector to store obstacles along with their distances
        std::vector<std::pair<double, joDetector::box3D>> obstaclesWithDistances;

        // Go through all obstacles and calculate distances
        for (const joDetector::box3D& bbox : this->dynamicBBoxes_) {
            Eigen::Vector3d obsPos(bbox.x, bbox.y, bbox.z);
            Eigen::Vector3d diff = currPos - obsPos;
            diff(2) = 0.;
            double distance = diff.norm();
            if (distance <= req->range) {
                obstaclesWithDistances.push_back(std::make_pair(distance, bbox));
            }
        }

        // Sort obstacles by distance in ascending order
        std::sort(obstaclesWithDistances.begin(), obstaclesWithDistances.end(), 
                [](const std::pair<double, joDetector::box3D>& a, const std::pair<double, joDetector::box3D>& b) {
                    return a.first < b.first;
                });

        // Push sorted obstacles into the response
        for (const auto& item : obstaclesWithDistances) {
            const joDetector::box3D& bbox = item.second;

            geometry_msgs::msg::Vector3 pos;
            geometry_msgs::msg::Vector3 vel;
            geometry_msgs::msg::Vector3 size;

            pos.x = bbox.x;
            pos.y = bbox.y;
            pos.z = bbox.z;

            vel.x = bbox.Vx;
            vel.y = bbox.Vy;
            vel.z = 0.;

            size.x = bbox.x_width;
            size.y = bbox.y_width;
            size.z = bbox.z_width;

            res->position.push_back(pos);
            res->velocity.push_back(vel);
            res->size.push_back(size);
        }

        return true;
    }

    void dynamicDetector::depthPoseCB(const sensor_msgs::msg::Image::ConstSharedPtr& img, const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose){

        // store current depth image
        cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
        if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1){
            (imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
        }
        imgPtr->image.copyTo(this->depthImage_);

        // store current position and orientation (camera)
        Eigen::Matrix4d camPoseDepthMatrix, camPoseColorMatrix;
        this->getCameraPose(pose, camPoseDepthMatrix, camPoseColorMatrix);

        this->position_(0) = pose->pose.position.x;
        this->position_(1) = pose->pose.position.y;
        this->position_(2) = pose->pose.position.z;
        Eigen::Quaterniond quat;
        quat = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z);
        Eigen::Matrix3d rot = quat.toRotationMatrix();
        this->orientation_ = rot;

        this->positionDepth_(0) = camPoseDepthMatrix(0, 3);
        this->positionDepth_(1) = camPoseDepthMatrix(1, 3);
        this->positionDepth_(2) = camPoseDepthMatrix(2, 3);
        this->orientationDepth_ = camPoseDepthMatrix.block<3, 3>(0, 0);

        this->positionColor_(0) = camPoseColorMatrix(0, 3);
        this->positionColor_(1) = camPoseColorMatrix(1, 3);
        this->positionColor_(2) = camPoseColorMatrix(2, 3);
        this->orientationColor_ = camPoseColorMatrix.block<3, 3>(0, 0);

        // Ground height now comes from wall_detector via /wall_detector/ground_height topic
    }

    void dynamicDetector::depthOdomCB(const sensor_msgs::msg::Image::ConstSharedPtr& img, const nav_msgs::msg::Odometry::ConstSharedPtr& odom){
        
        // store current depth image
        cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
        if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1){
            (imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
        }
        imgPtr->image.copyTo(this->depthImage_);

        // store current position and orientation (camera)
        Eigen::Matrix4d camPoseDepthMatrix, camPoseColorMatrix;
        this->getCameraPose(odom, camPoseDepthMatrix, camPoseColorMatrix);

        this->position_(0) = odom->pose.pose.position.x;
        this->position_(1) = odom->pose.pose.position.y;
        this->position_(2) = odom->pose.pose.position.z;
        Eigen::Quaterniond quat;
        quat = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
        Eigen::Matrix3d rot = quat.toRotationMatrix();
        this->orientation_ = rot;

        this->positionDepth_(0) = camPoseDepthMatrix(0, 3);
        this->positionDepth_(1) = camPoseDepthMatrix(1, 3);
        this->positionDepth_(2) = camPoseDepthMatrix(2, 3);
        this->orientationDepth_ = camPoseDepthMatrix.block<3, 3>(0, 0);

        this->positionColor_(0) = camPoseColorMatrix(0, 3);
        this->positionColor_(1) = camPoseColorMatrix(1, 3);
        this->positionColor_(2) = camPoseColorMatrix(2, 3);
        this->orientationColor_ = camPoseColorMatrix.block<3, 3>(0, 0);

        // Ground height now comes from wall_detector via /wall_detector/ground_height topic
    }

    void dynamicDetector::lidarPoseCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloudMsg, const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose){
        // for visualization
        this->latestCloud_ = cloudMsg;

        // local cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*cloudMsg, *tempCloud);

        // filter and downsample pointcloud
        // Create a filtered cloud pointer to store intermediate results
        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>());

        // Apply a pass-through filter to limit points to the local sensor range in X, Y, and Z axes
        pcl::PassThrough<pcl::PointXYZ> pass;

        // Filter for X axis
        pass.setInputCloud(tempCloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-this->localLidarRange_.x(), this->localLidarRange_.x());
        pass.filter(*filteredCloud);

        // Filter for Y axis
        pass.setInputCloud(filteredCloud);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-this->localLidarRange_.y(), this->localLidarRange_.y());
        pass.filter(*filteredCloud);

        int sigma = this->gaussianDownSampleRate_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr preTransformCloud(new pcl::PointCloud<pcl::PointXYZ>());
        preTransformCloud->reserve(filteredCloud->size());

        for (pcl::PointXYZ &pt : filteredCloud->points) {
            double dist = pow(pow(pt.x, 2) + pow(pt.y, 2), 0.5);
            double p = std::exp(-(dist * dist) / (2 * sigma * sigma));

            double r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            if (r < p) {
                preTransformCloud->push_back(pt);
            }
        }

        // Publish preTransformCloud in velodyne frame
        sensor_msgs::msg::PointCloud2 preTransformCloudMsg;
        pcl::toROSMsg(*preTransformCloud, preTransformCloudMsg);
        preTransformCloudMsg.header.frame_id = this->tfLidarFrame_;
        preTransformCloudMsg.header.stamp = cloudMsg->header.stamp;
        this->filteredLidarVelodynePub_->publish(preTransformCloudMsg);

        // transform
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.linear() = this->orientationLidar_;
        transform.translation() = this->positionLidar_;

        // map cloud
        // Create an empty point cloud to store the transformed data
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud (new pcl::PointCloud<pcl::PointXYZ>());

        // Apply the transformation
        pcl::transformPointCloud(*preTransformCloud, *transformedCloud, transform);

        // filter roof and ground 
        pcl::PointCloud<pcl::PointXYZ>::Ptr groundRoofFilterCloud (new pcl::PointCloud<pcl::PointXYZ>());
        pass.setInputCloud(transformedCloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(this->groundHeight_, this->roofHeight_);
        pass.filter(*groundRoofFilterCloud);

        // Filter out points inside wall bounding boxes
        {
            std::lock_guard<std::mutex> lock(this->wallBBoxesMutex_);
            if (!this->wallBBoxes_.empty()) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr wallFiltered(new pcl::PointCloud<pcl::PointXYZ>());
                wallFiltered->reserve(groundRoofFilterCloud->size());
                for (const auto& pt : groundRoofFilterCloud->points) {
                    if (!this->isInsideAnyWall(Eigen::Vector3d(pt.x, pt.y, pt.z))) {
                        wallFiltered->points.push_back(pt);
                    }
                }
                wallFiltered->width  = wallFiltered->points.size();
                wallFiltered->height = 1;
                groundRoofFilterCloud = wallFiltered;
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud = groundRoofFilterCloud;
        // Create the VoxelGrid filter object
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        // sor.setInputCloud(filteredCloud);
        sor.setInputCloud(groundRoofFilterCloud);

        // Set the leaf size (adjust to control the downsampling)
        sor.setLeafSize(0.1f, 0.1f, 0.1f); // Try different values based on your point cloud density

        // If the downsampled cloud has more than certain points, further increase the leaf size
        while (int(downsampledCloud->size()) > this->downSampleThresh_) {
            double leafSize = sor.getLeafSize().x() * 1.1f; // Increase the leaf size to reduce point count
            sor.setLeafSize(leafSize, leafSize, leafSize);
            sor.filter(*downsampledCloud);
        }

        this->lidarCloud_ = downsampledCloud;
        sensor_msgs::msg::PointCloud2 outputCloud;
        pcl::toROSMsg(*this->lidarCloud_, outputCloud); // Convert to ROS message
        outputCloud.header.frame_id = "odom";    // Set appropriate frame ID
        this->downSamplePointsPub_->publish(outputCloud);
        this->processedLidarPub_->publish(outputCloud);

        // store current position and orientation
        Eigen::Matrix4d lidarPoseMatrix;
        this->getLidarPose(pose, lidarPoseMatrix);

        this->position_(0) = pose->pose.position.x;
        this->position_(1) = pose->pose.position.y;
        this->position_(2) = pose->pose.position.z;
        Eigen::Quaterniond quat;
        quat = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z);
        Eigen::Matrix3d rot = quat.toRotationMatrix();
        this->orientation_ = rot;

        this->positionLidar_(0) = lidarPoseMatrix(0, 3);
        this->positionLidar_(1) = lidarPoseMatrix(1, 3);
        this->positionLidar_(2) = lidarPoseMatrix(2, 3);
        this->orientationLidar_ = lidarPoseMatrix.block<3, 3>(0, 0);
    }

    void dynamicDetector::lidarOdomCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloudMsg, const nav_msgs::msg::Odometry::ConstSharedPtr& odom){
        // for visualization
        this->latestCloud_ = cloudMsg;

        // local cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*cloudMsg, *tempCloud);

        // filter and downsample pointcloud
        // Create a filtered cloud pointer to store intermediate results
        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>());

        // Apply a pass-through filter to limit points to the local sensor range in X, Y, and Z axes
        pcl::PassThrough<pcl::PointXYZ> pass;

        // Filter for X axis
        pass.setInputCloud(tempCloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-this->localLidarRange_.x(), this->localLidarRange_.x());
        pass.filter(*filteredCloud);

        // Filter for Y axis
        pass.setInputCloud(filteredCloud);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-this->localLidarRange_.y(), this->localLidarRange_.y());
        pass.filter(*filteredCloud);

        int sigma = this->gaussianDownSampleRate_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr preTransformCloud(new pcl::PointCloud<pcl::PointXYZ>());
        preTransformCloud->reserve(filteredCloud->size());

        for (pcl::PointXYZ &pt : filteredCloud->points) {
            double dist = pow(pow(pt.x, 2) + pow(pt.y, 2), 0.5);
            double p = std::exp(-(dist * dist) / (2 * sigma * sigma));

            double r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            if (r < p) {
                preTransformCloud->push_back(pt);
            }
        }

        // Publish preTransformCloud in velodyne frame
        sensor_msgs::msg::PointCloud2 preTransformCloudMsg;
        pcl::toROSMsg(*preTransformCloud, preTransformCloudMsg);
        preTransformCloudMsg.header.frame_id = this->tfLidarFrame_;
        preTransformCloudMsg.header.stamp = cloudMsg->header.stamp;
        this->filteredLidarVelodynePub_->publish(preTransformCloudMsg);

        // transform
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.linear() = this->orientationLidar_;
        transform.translation() = this->positionLidar_;

        // map cloud
        // Create an empty point cloud to store the transformed data
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud (new pcl::PointCloud<pcl::PointXYZ>());

        // Apply the transformation
        pcl::transformPointCloud(*preTransformCloud, *transformedCloud, transform);

        // filter roof and ground 
        pcl::PointCloud<pcl::PointXYZ>::Ptr groundRoofFilterCloud (new pcl::PointCloud<pcl::PointXYZ>());
        pass.setInputCloud(transformedCloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(this->groundHeight_, this->roofHeight_);
        pass.filter(*groundRoofFilterCloud);

        // Filter out points inside wall bounding boxes
        {
            std::lock_guard<std::mutex> lock(this->wallBBoxesMutex_);
            if (!this->wallBBoxes_.empty()) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr wallFiltered(new pcl::PointCloud<pcl::PointXYZ>());
                wallFiltered->reserve(groundRoofFilterCloud->size());
                for (const auto& pt : groundRoofFilterCloud->points) {
                    if (!this->isInsideAnyWall(Eigen::Vector3d(pt.x, pt.y, pt.z))) {
                        wallFiltered->points.push_back(pt);
                    }
                }
                wallFiltered->width  = wallFiltered->points.size();
                wallFiltered->height = 1;
                groundRoofFilterCloud = wallFiltered;
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud = groundRoofFilterCloud;
        // Create the VoxelGrid filter object
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        // sor.setInputCloud(filteredCloud);
        sor.setInputCloud(groundRoofFilterCloud);

        // Set the leaf size (adjust to control the downsampling)
        sor.setLeafSize(0.1f, 0.1f, 0.1f); // Try different values based on your point cloud density

        // If the downsampled cloud has more than certain points, further increase the leaf size
        while (int(downsampledCloud->size()) > this->downSampleThresh_) {
            double leafSize = sor.getLeafSize().x() * 1.1f; // Increase the leaf size to reduce point count
            sor.setLeafSize(leafSize, leafSize, leafSize);
            sor.filter(*downsampledCloud);
        }

        this->lidarCloud_ = downsampledCloud;
        sensor_msgs::msg::PointCloud2 outputCloud;
        pcl::toROSMsg(*this->lidarCloud_, outputCloud); // Convert to ROS message
        outputCloud.header.frame_id = "odom";    // Set appropriate frame ID
        this->downSamplePointsPub_->publish(outputCloud);
        this->processedLidarPub_->publish(outputCloud);
        
        // store current position and orientation
        Eigen::Matrix4d lidarPoseMatrix;
        this->getLidarPose(odom, lidarPoseMatrix);

        this->position_(0) = odom->pose.pose.position.x;
        this->position_(1) = odom->pose.pose.position.y;
        this->position_(2) = odom->pose.pose.position.z;
        Eigen::Quaterniond quat;
        quat = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
        Eigen::Matrix3d rot = quat.toRotationMatrix();
        this->orientation_ = rot;

        this->positionLidar_(0) = lidarPoseMatrix(0, 3);
        this->positionLidar_(1) = lidarPoseMatrix(1, 3);
        this->positionLidar_(2) = lidarPoseMatrix(2, 3);
        this->orientationLidar_ = lidarPoseMatrix.block<3, 3>(0, 0);
    }

    void dynamicDetector::colorImgCB(const sensor_msgs::msg::Image::ConstSharedPtr& img){
        cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
        
        imgPtr->image.copyTo(this->detectedColorImage_);
    }

    void dynamicDetector::yoloDetectionCB(const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections){

        if(this->lidarToDepthCamOk_ == false){
            RCLCPP_WARN(this->nh_->get_logger(), "[dynamicDetector]: Waiting for LiDAR to depth camera extrinsic calibration...");
            return;
        }

        this->yoloDetectionResults_ = *detections;
    }

    void dynamicDetector::groundHeightCB(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg){
        if (msg->data.size() >= 2) {
            this->groundHeight_ = msg->data[0];
            this->roofHeight_   = msg->data[1];
            // std::cout << "[dynamicDetector] groundHeightCB: ground=" << this->groundHeight_
            //           << "  roof=" << this->roofHeight_ << std::endl;
            if (!this->groundEstimated_) {
                this->groundEstimated_ = true;
                RCLCPP_INFO(this->nh_->get_logger(),
                    "[dynamicDetector] Received ground=%.3f roof=%.3f from wall_detector",
                    this->groundHeight_, this->roofHeight_);
            }
        } else {
            // std::cout << "[dynamicDetector] groundHeightCB: received msg with "
            //           << msg->data.size() << " elements (expected >=2)" << std::endl;
        }
    }

    void dynamicDetector::wallMarkersCB(const visualization_msgs::msg::MarkerArray::ConstSharedPtr& msg){
        std::vector<WallOBB> new_walls;
        new_walls.reserve(msg->markers.size());

        for (const auto& m : msg->markers) {
            // Only process CUBE markers (wall bounding boxes)
            if (m.type != visualization_msgs::msg::Marker::CUBE) continue;
            if (m.action == visualization_msgs::msg::Marker::DELETE) continue;

            WallOBB wall;
            wall.center = Eigen::Vector3d(
                m.pose.position.x,
                m.pose.position.y,
                m.pose.position.z);
            Eigen::Quaterniond q(
                m.pose.orientation.w,
                m.pose.orientation.x,
                m.pose.orientation.y,
                m.pose.orientation.z);
            wall.rotation  = q.toRotationMatrix();
            wall.half_size = Eigen::Vector3d(
                m.scale.x * 0.5,
                m.scale.y * 0.5,
                m.scale.z * 0.5);

            new_walls.push_back(wall);
        }

        std::lock_guard<std::mutex> lock(this->wallBBoxesMutex_);
        this->wallBBoxes_ = std::move(new_walls);
        // std::cout << "[dynamicDetector] wallMarkersCB: received " << this->wallBBoxes_.size()
        //           << " wall bounding boxes" << std::endl;
        // for (size_t i = 0; i < this->wallBBoxes_.size(); ++i) {
        //     const auto& w = this->wallBBoxes_[i];
        //     std::cout << "  wall[" << i << "] center=(" << w.center.x() << ", "
        //               << w.center.y() << ", " << w.center.z() << ") half_size=("
        //               << w.half_size.x() << ", " << w.half_size.y() << ", "
        //               << w.half_size.z() << ")" << std::endl;
        // }
    }
   
    void dynamicDetector::mainCB(){

        if(this->lidarToDepthCamOk_ == false){
            RCLCPP_WARN(this->nh_->get_logger(), "[dynamicDetector]: Waiting for LiDAR to depth camera extrinsic calibration...");
            return;
        }

        // --- parallel detection: dbscan + uv + lidar ---
        std::thread t_db([this]{ this->dbscanDetect(); });
        std::thread t_uv([this]{ this->uvDetect(); });
        std::thread t_li([this]{ this->lidarDetect(); });
        t_db.join();
        t_uv.join();
        t_li.join();

        this->filterLVBBoxes();
        this->newDetectFlag_ = true;

        // --- tracking ---
        // Filter out bboxes whose bottom face is too high above ground
        {
            const double zminThresh = this->groundHeight_ + this->trackMaxZminAboveGround_;
            std::vector<joDetector::box3D> keptBBoxes;
            std::vector<std::vector<Eigen::Vector3d>> keptPc;
            std::vector<Eigen::Vector3d> keptCenters;
            std::vector<Eigen::Vector3d> keptStds;
            keptBBoxes.reserve(this->filteredBBoxes_.size());
            keptPc.reserve(this->filteredPcClusters_.size());
            keptCenters.reserve(this->filteredPcClusterCenters_.size());
            keptStds.reserve(this->filteredPcClusterStds_.size());
            for (size_t i = 0; i < this->filteredBBoxes_.size(); ++i) {
                const auto& b = this->filteredBBoxes_[i];
                double zmin = b.z - b.z_width / 2.0;
                if (zmin <= zminThresh) {
                    keptBBoxes.push_back(b);
                    if (i < this->filteredPcClusters_.size())
                        keptPc.push_back(this->filteredPcClusters_[i]);
                    if (i < this->filteredPcClusterCenters_.size())
                        keptCenters.push_back(this->filteredPcClusterCenters_[i]);
                    if (i < this->filteredPcClusterStds_.size())
                        keptStds.push_back(this->filteredPcClusterStds_[i]);
                }
            }
            this->filteredBBoxes_ = std::move(keptBBoxes);
            this->filteredPcClusters_ = std::move(keptPc);
            this->filteredPcClusterCenters_ = std::move(keptCenters);
            this->filteredPcClusterStds_ = std::move(keptStds);
        }

        // Debug: Print history and filter sizes before tracking
        // RCLCPP_INFO(this->nh_->get_logger(), "[DEBUG] boxHist_ size: %zu, pcHist_ size: %zu, filters_ size: %zu, missedFrames_ size: %zu, trackedBBoxes_ size: %zu", this->boxHist_.size(), this->pcHist_.size(), this->filters_.size(), this->missedFrames_.size(), this->trackedBBoxes_.size());

        std::vector<int> bestMatch;

        this->boxAssociation(bestMatch);

        if (!this->boxHist_.empty() || !this->filteredBBoxes_.empty()){
            this->kalmanFilterAndUpdateHist(bestMatch);
            this->populatePredictionVisualizationBBoxes();
        }
        else{
            this->boxHist_.clear();
            this->pcHist_.clear();
            this->pcCenterHist_.clear();
            this->filters_.clear();
            this->missedFrames_.clear();
            this->trackedBBoxes_.clear();
            this->predictedBBoxesActive_.clear();
            this->predictedBBoxesMissed_.clear();
            this->predictedBBoxesUnconfirmed_.clear();
        }

        // Debug: Print history and filter sizes after tracking
        // RCLCPP_INFO(this->nh_->get_logger(), "[DEBUG] (post) boxHist_ size: %zu, pcHist_ size: %zu, filters_ size: %zu, missedFrames_ size: %zu, trackedBBoxes_ size: %zu", this->boxHist_.size(), this->pcHist_.size(), this->filters_.size(), this->missedFrames_.size(), this->trackedBBoxes_.size());

        // --- classification ---
        // Only classify tracks that are currently in the tracked output
        // (confirmed + not stationary-suppressed + not predict-only).
        // This guarantees every dynamic (blue) bbox has a corresponding tracked (yellow) bbox.
        std::unordered_set<int> outputTrackIds;
        for (const auto& tb : this->trackedBBoxes_){
            outputTrackIds.insert(static_cast<int>(tb.id));
        }

        std::vector<joDetector::box3D> dynamicBBoxesTemp;
        std::vector<joDetector::box3D> potentiallyDynamicBBoxesTemp;

        for (size_t i=0; i<this->pcHist_.size() ; ++i){

            // Skip tracks not present in the tracked output.
            if (this->boxHist_[i].empty() ||
                outputTrackIds.find(static_cast<int>(this->boxHist_[i][0].id)) == outputTrackIds.end()){
                continue;
            }

            // ===================================================================================
            // PATH 1 — YOLO: classification based on KF speed.
            // YOLO-certified tracks with speed >= dynaVelThresh_ are dynamic.
            // YOLO-certified tracks with speed ~ 0 are "potentially dynamic" (stationary
            // recognized object — e.g. a person standing still). is_dynamic is written into
            // boxHist_ only for moving YOLO tracks so PATH 2 can sustain them after they
            // exit the camera FOV.
            // ===================================================================================
            if (this->boxHist_[i][0].is_yolo_candidate){
                const double kfSpeed = std::sqrt(this->boxHist_[i][0].Vx * this->boxHist_[i][0].Vx +
                                                 this->boxHist_[i][0].Vy * this->boxHist_[i][0].Vy);
                if (kfSpeed >= this->dynaVelThresh_){
                    this->boxHist_[i][0].is_dynamic = true;
                    dynamicBBoxesTemp.push_back(this->boxHist_[i][0]);
                } else {
                    this->boxHist_[i][0].is_potentially_dynamic = true;
                    potentiallyDynamicBBoxesTemp.push_back(this->boxHist_[i][0]);
                }
                continue;
            }

            const Eigen::Vector3d currBoxPos(this->boxHist_[i][0].x,
                                            this->boxHist_[i][0].y,
                                            this->boxHist_[i][0].z);

            const bool outsideFovNonYolo = !this->isInCameraFOV(currBoxPos) &&
                                        !this->boxHist_[i][0].is_yolo_candidate;

            bool outsideFovObservedMotionOk = true;

            double outsideNetDisp = 0.0;
            double outsideNetSpeed = 0.0;
            double outsidePathLen = 0.0;
            double outsideStraightness = 0.0;
            double outsideMaxStepSpeed = 0.0;

            if (outsideFovNonYolo){
                if (static_cast<int>(this->boxHist_[i].size()) < this->outsideFovClassMinTrackAge_){
                    outsideFovObservedMotionOk = false;
                }
                else{
                    outsideFovObservedMotionOk =
                        this->hasConsistentObservedMotionOutsideFov(
                            static_cast<int>(i),
                            this->outsideFovClassWindow_,
                            outsideNetDisp,
                            outsideNetSpeed,
                            outsidePathLen,
                            outsideStraightness,
                            outsideMaxStepSpeed);
                }
            }
            // -----------------------------------------------------------------------------------
            // Determine the temporal baseline for motion analysis (curFrameGap).
            // Ideally skipFrame_ frames back (e.g. 2 → 0.24s), but use whatever history is
            // available if the track is still young.
            // -----------------------------------------------------------------------------------
            int curFrameGap;
            if (int(this->pcHist_[i].size()) < this->skipFrame_+1){
                curFrameGap = this->pcHist_[i].size() - 1;
            }
            else{
                curFrameGap = this->skipFrame_;
            }

            // ===================================================================================
            // PATH 2 — Historical continuity (shortcut for already-dynamic tracks).
            // If enough of the last forceDynaCheckRange_ frames were classified is_dynamic,
            // re-confirm immediately — but ONLY if the track is still moving (KF speed ≥ thresh).
            // A track that stops loses the label even if it was dynamic before.
            // History entries [1..N] reflect classificationCB decisions from previous frames
            // (is_dynamic is non-sticky, so each entry is an independent decision).
            // ===================================================================================
            int dynaFrames = 0;
            if (int(this->boxHist_[i].size()) > this->forceDynaCheckRange_){
                for (int j=1 ; j<this->forceDynaCheckRange_+1 ; ++j){
                    if (this->boxHist_[i][j].is_dynamic){
                        ++dynaFrames;
                    }
                }
            }

            if (dynaFrames >= this->forceDynaFrames_){
                const double kfSpeed = std::sqrt(this->boxHist_[i][0].Vx * this->boxHist_[i][0].Vx +
                                                this->boxHist_[i][0].Vy * this->boxHist_[i][0].Vy);

                bool keepDynamic = (kfSpeed >= this->dynaVelThresh_);

                // Fuori FOV non-YOLO: non basta che il KF dica "sto andando".
                // Serve anche moto osservato coerente, per evitare statici rumorosi.
                if (outsideFovNonYolo){
                    keepDynamic = keepDynamic && outsideFovObservedMotionOk;
                }

                if (keepDynamic){
                    this->boxHist_[i][0].is_dynamic = true;
                    dynamicBBoxesTemp.push_back(this->boxHist_[i][0]);
                    continue;
                }
            }

            // ===================================================================================
            // PATH 3 — Point-cloud motion voting.
            // The full analysis path for tracks that are not YOLO and not already dynamic.
            // Consists of 3 stages:
            //   3a) Kalman velocity confidence (alternative to voting for sparse LiDAR clouds)
            //   3b) Per-point nearest-neighbor voting
            //   3c) Multi-frame consistency check (anti-flicker)
            // ===================================================================================

            // --- Velocity setup ---
            std::vector<Eigen::Vector3d> currPc = this->pcHist_[i][0];
            std::vector<Eigen::Vector3d> prevPc = this->pcHist_[i][curFrameGap];
            Eigen::Vector3d Vcur(0.,0.,0.);  // per-point velocity (reused in loop)
            Eigen::Vector3d Vbox(0.,0.,0.);  // box centroid velocity (finite difference)
            Eigen::Vector3d Vkf(0.,0.,0.);   // Kalman filter velocity estimate
            int numPoints = currPc.size();    // effective point count (decremented for outliers)
            int votes = 0;                    // points voting "dynamic"

            // Box velocity: finite difference of centroid position over curFrameGap frames.
            Vbox(0) = (this->boxHist_[i][0].x - this->boxHist_[i][curFrameGap].x)/(this->dt_*curFrameGap);
            Vbox(1) = (this->boxHist_[i][0].y - this->boxHist_[i][curFrameGap].y)/(this->dt_*curFrameGap);
            Vbox(2) = (this->boxHist_[i][0].z - this->boxHist_[i][curFrameGap].z)/(this->dt_*curFrameGap);
            Vkf(0) = this->boxHist_[i][0].Vx;
            Vkf(1) = this->boxHist_[i][0].Vy;

            // --- Stage 3a: Kalman velocity confidence ---
            // Outside the camera FOV, the point cloud comes only from LiDAR (sparse),
            // so per-point voting may be unreliable. As an alternative, check whether
            // the Kalman filter has been consistently estimating high speed with low
            // variance over recent frames. If so, treat it as confident motion evidence
            // that can bypass the point-voting requirement.
            int kfSamples = std::min(curFrameGap + 1, (int)this->boxHist_[i].size());
            double kfVelMean = 0.0;
            for (int h = 0; h < kfSamples; ++h){
                kfVelMean += std::sqrt(this->boxHist_[i][h].Vx * this->boxHist_[i][h].Vx +
                                       this->boxHist_[i][h].Vy * this->boxHist_[i][h].Vy);
            }
            kfVelMean /= std::max(kfSamples, 1);

            double kfVelVar = 0.0;
            for (int h = 0; h < kfSamples; ++h){
                double v = std::sqrt(this->boxHist_[i][h].Vx * this->boxHist_[i][h].Vx +
                                     this->boxHist_[i][h].Vy * this->boxHist_[i][h].Vy);
                kfVelVar += (v - kfVelMean) * (v - kfVelMean);
            }
            kfVelVar /= std::max(kfSamples, 1);
            const double kfVelStd = std::sqrt(kfVelVar);

            // Confident = mean speed above threshold AND std small relative to mean.
            const bool kfVelConfident = (kfVelMean >= this->dynaVelThresh_) &&
                                        (kfVelStd <= this->dynaKfVelStdRatio_ * kfVelMean);

            // --- Stage 3b: Per-point nearest-neighbor voting ---
            // For each point in the current cloud, find its nearest neighbor in the
            // cloud from curFrameGap frames ago. Compute the per-point velocity and
            // check its alignment (cosine similarity) with the box velocity:
            //   - velSim < 0  → point moves opposite to the box → exclude from count
            //   - velSim >= 0 AND |Vcur| > threshold → vote "dynamic"
            for (size_t j=0 ; j<currPc.size() ; ++j){
                double minDist = 2;
                Eigen::Vector3d nearestVect;
                for (size_t k=0 ; k<prevPc.size() ; k++){
                    double dist = (currPc[j]-prevPc[k]).norm();
                    if (abs(dist) < minDist){
                        minDist = dist;
                        nearestVect = currPc[j]-prevPc[k];
                    }
                }
                Vcur = nearestVect/(this->dt_*curFrameGap); Vcur(2) = 0;
                double velSim = Vcur.dot(Vbox)/(Vcur.norm()*Vbox.norm());

                if (velSim < 0){
                    --numPoints;
                }
                else{
                    if (Vcur.norm()>this->dynaVelThresh_){
                        ++votes;
                    }
                }
            }

            // --- Stage 3c: Decision + multi-frame consistency check ---
            double voteRatio = (numPoints>0)?double(votes)/double(numPoints):0;
            double velNorm = Vkf.norm();

            // Dynamic candidate condition (single-frame):
            //   Hard gate: KF speed must be above dynaVelThresh_ (always required).
            //   Soft gate: EITHER enough points voted dynamic (voteRatio >= dynaVoteThresh_)
            //              OR the KF velocity is confident (handles sparse outside-FOV clouds).
            bool singleFrameDynamicEvidence =
                (voteRatio >= this->dynaVoteThresh_ || kfVelConfident) &&
                velNorm >= this->dynaVelThresh_;

            // Fuori FOV non-YOLO: richiedi anche moto osservato multi-frame.
            // Questo evita che jitter di cluster statici venga promosso a dinamico.
            if (outsideFovNonYolo){
                singleFrameDynamicEvidence =
                    singleFrameDynamicEvidence && outsideFovObservedMotionOk;
            }

            if (singleFrameDynamicEvidence){
                this->boxHist_[i][0].is_dynamic_candidate = true;

                // Anti-flicker consistency check: require dynamicConsistThresh_ consecutive
                // frames of dynamic evidence before actually classifying as dynamic.
                // Evidence = any of: is_dynamic_candidate, is_yolo_candidate, or is_dynamic.
                int dynaConsistCount = 0;
                if (int(this->boxHist_[i].size()) >= this->dynamicConsistThresh_){
                    for (int j=0 ; j<this->dynamicConsistThresh_; ++j){
                        if (this->boxHist_[i][j].is_dynamic_candidate or this->boxHist_[i][j].is_yolo_candidate or this->boxHist_[i][j].is_dynamic){
                            ++dynaConsistCount;
                        }
                    }
                }
                // All N frames have evidence → classify as dynamic.
                if (dynaConsistCount == this->dynamicConsistThresh_){
                    this->boxHist_[i][0].is_dynamic = true;
                    dynamicBBoxesTemp.push_back(this->boxHist_[i][0]);
                }
            }
        }

        // Optional post-filter: discard dynamic bboxes whose dimensions are too far
        // from the expected target object sizes (e.g. person ≈ 0.5×0.5×1.7 m).
        if (this->constrainSize_){
            std::vector<joDetector::box3D> dynamicBBoxesBeforeConstrain = dynamicBBoxesTemp;
            dynamicBBoxesTemp.clear();

            for (joDetector::box3D ob : dynamicBBoxesBeforeConstrain){
                bool findMatch = false;
                for (Eigen::Vector3d targetSize : this->targetObjectSize_){
                    double xdiff = std::abs(ob.x_width - targetSize(0));
                    double ydiff = std::abs(ob.y_width - targetSize(1));
                    double zdiff = std::abs(ob.z_width - targetSize(2)); 
                    if (xdiff < 0.8 and ydiff < 0.8 and zdiff < 1.0){
                        findMatch = true;
                    }
                }

                if (findMatch){
                    dynamicBBoxesTemp.push_back(ob);
                }
            }
        }

        this->dynamicBBoxes_ = dynamicBBoxesTemp;
        this->potentiallyDynamicBBoxes_ = potentiallyDynamicBBoxesTemp;

        // trackedBBoxes_ is assembled before classification in the same mainCB()
        // cycle. Refresh its classification flags now so visualization and
        // ObstacleArray publication use the current classification, not the
        // previous history snapshot.
        for (auto& trackedBox : this->trackedBBoxes_){
            trackedBox.is_dynamic = false;
            trackedBox.is_potentially_dynamic = false;

            for (const auto& hist : this->boxHist_){
                if (hist.empty()) continue;
                const auto& classifiedBox = hist.front();
                if (classifiedBox.id != trackedBox.id) continue;

                trackedBox.is_dynamic = classifiedBox.is_dynamic;
                trackedBox.is_potentially_dynamic = classifiedBox.is_potentially_dynamic;
                break;
            }
        }
    }

    void dynamicDetector::visCB(){
        this->publishUVImages();
        this->publishColorImages();
        
        this->publish3dBox(this->uvBBoxes_, this->uvBBoxesPub_, 0, 1, 0);
        this->publish3dBox(this->dbBBoxes_, this->dbBBoxesPub_, 1, 0, 0);
        this->publish3dBox(this->visualBBoxes_, this->visualBBoxesPub_, 0.3, 0.8, 1.0);
        this->publish3dBox(this->lidarBBoxes_, this->lidarBBoxesPub_, 0.5, 0.5, 0.5);
        this->publish3dBox(this->filteredBBoxesBeforeYolo_, this->filteredBBoxesBeforeYoloPub_, 0, 1, 0.5);
        this->publish3dBox(this->filteredBBoxes_, this->filteredBBoxesPub_, 0, 1, 1, 1.0, true);

        this->refreshTrackedBBoxClassificationFlags();
        const double trackedMarkerLifetime = std::max(0.25, 2.5 * this->dt_);
        this->publish3dBox(this->trackedBBoxes_, this->trackedBBoxesPub_, 0.74, 0.56, 0.18, 1.0, false, true, trackedMarkerLifetime);
        this->publish3dBox(this->predictedBBoxesActive_,       this->predictedBBoxesActivePub_,       0.72, 0.30, 0.05, 0.55, false, false);
        this->publish3dBox(this->predictedBBoxesMissed_,       this->predictedBBoxesMissedPub_,       1.0, 0.15, 0.15, 0.6, false, false);
        this->publish3dBox(this->predictedBBoxesUnconfirmed_,  this->predictedBBoxesUnconfirmedPub_,  0.6, 0.6, 0.6, 0.35, false, false);
        // Publish track IDs as text markers above each tracked bbox
        visualization_msgs::msg::MarkerArray trackIdMarkers;
        for (size_t i = 0; i < this->trackedBBoxes_.size(); ++i) {
            visualization_msgs::msg::Marker idMarker;
            idMarker.header.frame_id = "odom";
            idMarker.header.stamp = this->nh_->now();
            idMarker.ns = "track_id";
            idMarker.id = i;
            idMarker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            idMarker.action = visualization_msgs::msg::Marker::ADD;
            // Position marker just above the bbox, slightly offset
            idMarker.pose.position.x = this->trackedBBoxes_[i].x;
            idMarker.pose.position.y = this->trackedBBoxes_[i].y;
            idMarker.pose.position.z = this->trackedBBoxes_[i].z + this->trackedBBoxes_[i].z_width/2. + 0.05;
            // Make text smaller
            idMarker.scale.z = 0.15;
            idMarker.scale.x = 0.15;
            idMarker.scale.y = 0.15;
            idMarker.color.a = 1.0;
            if (this->trackedBBoxes_[i].is_dynamic){
                idMarker.color.r = 0.0;
                idMarker.color.g = 0.0;
                idMarker.color.b = 1.0;
            } else if (this->trackedBBoxes_[i].is_potentially_dynamic){
                idMarker.color.r = 0.0;
                idMarker.color.g = 1.0;
                idMarker.color.b = 0.0;
            } else {
                idMarker.color.r = 0.0;
                idMarker.color.g = 1.0;
                idMarker.color.b = 0.0;
            }
            int track_id = static_cast<int>(this->trackedBBoxes_[i].id);
            double vx = this->trackedBBoxes_[i].Vx;
            double vy = this->trackedBBoxes_[i].Vy;
            double vnorm = std::sqrt(vx*vx + vy*vy);
            const std::string statusStr =
                this->trackedBBoxes_[i].is_dynamic            ? "dynamic" :
                this->trackedBBoxes_[i].is_potentially_dynamic ? "potentially_dynamic" :
                (vnorm >= this->dynaVelThresh_)                ? "tracked" : "static";
            idMarker.text = "ID: " + std::to_string(track_id)
                          + " | age: " + std::to_string(this->trackedBBoxes_[i].track_age)
                          + " | |V|: " + std::to_string(vnorm).substr(0, 4)
                          + " | " + statusStr;
            idMarker.lifetime = rclcpp::Duration::from_seconds(trackedMarkerLifetime);
            trackIdMarkers.markers.push_back(idMarker);
        }
        this->trackedBBoxesPub_->publish(trackIdMarkers);
        this->publish3dBox(this->dynamicBBoxes_, this->dynamicBBoxesPub_, 0, 0, 1);
        this->publish3dBox(this->potentiallyDynamicBBoxes_, this->potentiallyDynamicBBoxesPub_, 0.0, 1.0, 0.0);

        this->publishLidarClusters(); // colored clusters
        this->publishFilteredPoints();
        std::vector<Eigen::Vector3d> dynamicPoints;
        this->getDynamicPc(dynamicPoints);
        this->publishPoints(dynamicPoints, this->dynamicPointsPub_);
        this->publishPoints(this->filteredDepthPoints_, this->filteredDepthPointsPub_);
        this->publishRawDynamicPoints();

        this->publishHistoryTraj();
        this->publishVelVis();
        this->publishDynamicObstacleArray();
        this->publishTrackedDynamicObstacleMarkers();
    }

    void dynamicDetector::publishDynamicObstacleArray(){
        this->refreshTrackedBBoxClassificationFlags();

        jo_msgs::msg::ObstacleArray arr;
        arr.header.frame_id = "odom";
        arr.header.stamp = this->nh_->now();

        for (const auto& box : this->trackedBBoxes_){
            jo_msgs::msg::Obstacle obs;
            obs.header   = arr.header;
            obs.track_id = static_cast<uint32_t>(box.id);
            obs.track_age = box.track_age;

            obs.pose.position.x  = box.x;
            obs.pose.position.y  = box.y;
            obs.pose.position.z  = box.z;
            obs.pose.orientation.w = 1.0;

            obs.size.x = box.x_width;
            obs.size.y = box.y_width;
            obs.size.z = box.z_width;

            obs.twist.linear.x = box.Vx;
            obs.twist.linear.y = box.Vy;
            obs.twist.linear.z = box.Vz;

            obs.accel.linear.x = box.Ax;
            obs.accel.linear.y = box.Ay;
            obs.accel.linear.z = box.Az;

            // Determine classification status (priority order)
            const double kfSpeed = std::sqrt(box.Vx * box.Vx + box.Vy * box.Vy);
            if (box.is_dynamic){
                obs.status = jo_msgs::msg::Obstacle::STATUS_DYNAMIC;
            } else if (box.is_potentially_dynamic){
                obs.status = jo_msgs::msg::Obstacle::STATUS_POTENTIALLY_DYNAMIC;
            } else if (kfSpeed >= this->dynaVelThresh_){
                obs.status = jo_msgs::msg::Obstacle::STATUS_TRACKED;
            } else {
                obs.status = jo_msgs::msg::Obstacle::STATUS_STATIC;
            }

            arr.obstacles.push_back(obs);
        }

        this->trackedObstaclesPub_->publish(arr);
    }

    void dynamicDetector::publishTrackedDynamicObstacleMarkers(){
        std::vector<joDetector::box3D> obstacleBoxes;
        obstacleBoxes.reserve(this->trackedBBoxes_.size());

        for (const auto& box : this->trackedBBoxes_){
            obstacleBoxes.push_back(box);
        }

        // This mirrors /tracked_dynamic_obstacles for RViz. Dynamic and
        // potentially dynamic boxes keep their semantic colors through
        // publish3dBox(); plain TRACKED obstacles use a muted amber fallback.
        this->publish3dBox(obstacleBoxes, this->trackedObstaclesBBoxesPub_, 0.85, 0.55, 0.18);
    }

    void dynamicDetector::refreshTrackedBBoxClassificationFlags(){
        std::unordered_set<int> dynamicTrackIds;
        std::unordered_set<int> potentiallyDynamicTrackIds;

        for (const auto& box : this->dynamicBBoxes_){
            dynamicTrackIds.insert(static_cast<int>(box.id));
        }
        for (const auto& box : this->potentiallyDynamicBBoxes_){
            potentiallyDynamicTrackIds.insert(static_cast<int>(box.id));
        }
        for (const auto& hist : this->boxHist_){
            if (hist.empty()) continue;
            const auto& box = hist.front();
            const int trackId = static_cast<int>(box.id);
            if (box.is_dynamic){
                dynamicTrackIds.insert(trackId);
            }
            if (box.is_potentially_dynamic){
                potentiallyDynamicTrackIds.insert(trackId);
            }
        }

        for (auto& box : this->trackedBBoxes_){
            const int trackId = static_cast<int>(box.id);
            box.is_dynamic = dynamicTrackIds.find(trackId) != dynamicTrackIds.end();
            box.is_potentially_dynamic =
                !box.is_dynamic &&
                potentiallyDynamicTrackIds.find(trackId) != potentiallyDynamicTrackIds.end();
        }
    }

    void dynamicDetector::uvDetect(){
        // initialization
        if (this->uvDetector_ == NULL){
            // RCLCPP_INFO(this->nh_->get_logger(), "[DEBUG] Allocating UVdetector with fx: %f, fy: %f, px: %f, py: %f, depthScale_: %f, max_dist: %f", this->fx_, this->fy_, this->cx_, this->cy_, this->depthScale_, this->raycastMaxLength_ * 1000);
            this->uvDetector_.reset(new UVdetector ());
            this->uvDetector_->fx = this->fx_;
            this->uvDetector_->fy = this->fy_;
            this->uvDetector_->px = this->cx_;
            this->uvDetector_->py = this->cy_;
            this->uvDetector_->depthScale_ = this->depthScale_; 
            this->uvDetector_->max_dist = this->raycastMaxLength_ * 1000;
        }

        // detect from depth mapcalBox
        if (not this->depthImage_.empty()){
            this->uvDetector_->depth = this->depthImage_;
            this->uvDetector_->detect();
            this->uvDetector_->extract_3Dbox();
            // std::cout << "[DEBUG] After extract_3Dbox: bounding_box_U size = " << this->uvDetector_->bounding_box_U.size() << ", bounding_box_D size = " << this->uvDetector_->bounding_box_D.size() << ", box3Ds size = " << this->uvDetector_->box3Ds.size() << std::endl;

            this->uvDetector_->display_U_map();
            this->uvDetector_->display_bird_view();
            this->uvDetector_->display_depth();

            // transform to the world frame (recalculate the bounding boxes)
            // std::cout << "[DEBUG] Before transformUVBBoxes, uvBBoxes_ size = " << this->uvBBoxes_.size() << std::endl;
            std::vector<joDetector::box3D> uvBBoxes;
            this->transformUVBBoxes(uvBBoxes);
            // std::cout << "[DEBUG] After transformUVBBoxes, uvBBoxes size = " << uvBBoxes.size() << std::endl;
            this->uvBBoxes_ = uvBBoxes;

            // Filter UV bboxes — must happen here, after uvBBoxes_ is assigned,
            // NOT in dbscanDetect() where it ran on the previous frame's data.
            std::vector<joDetector::box3D> filteredUVBBoxes;
            for (const auto& box : this->uvBBoxes_) {
                if (!this->isOversizedBBox(box)) {
                    filteredUVBBoxes.push_back(box);
                }
            }
            this->uvBBoxes_ = filteredUVBBoxes;
        }
    }

    bool dynamicDetector::isOversizedBBox(const joDetector::box3D& bbox) const {
        const double xyDiag = std::sqrt(bbox.x_width * bbox.x_width +
                                        bbox.y_width * bbox.y_width);
        const double volume = bbox.x_width * bbox.y_width * bbox.z_width;
        return xyDiag > this->uvBoxMaxDiagonal_ ||
               bbox.z_width > this->boxMaxHeight_ ||
               volume > this->boxMaxVolume_;
    }

    void dynamicDetector::dbscanDetect(){
        // 1. get pointcloud
        // RCLCPP_INFO(this->nh_->get_logger(), "[DEBUG] Calling projectDepthImage()");
        this->projectDepthImage();

        // 2. filter points
        // RCLCPP_INFO(this->nh_->get_logger(), "[DEBUG] Filtering projPoints_ (size: %zu) to filteredDepthPoints_", this->projPoints_.size());
        this->filterPoints(this->projPoints_, this->filteredDepthPoints_);

        // 3. cluster points and get bounding boxes
        // RCLCPP_INFO(this->nh_->get_logger(), "[DEBUG] Clustering filteredDepthPoints_ (size: %zu)", this->filteredDepthPoints_.size());
        this->clusterPointsAndBBoxes(this->filteredDepthPoints_, this->dbBBoxes_, this->pcClustersVisual_,
                                     this->pcClusterCentersVisual_, this->pcClusterStdsVisual_);

        {
            std::vector<joDetector::box3D> keptBBoxes;
            std::vector<std::vector<Eigen::Vector3d>> keptClusters;
            std::vector<Eigen::Vector3d> keptCenters;
            std::vector<Eigen::Vector3d> keptStds;
            keptBBoxes.reserve(this->dbBBoxes_.size());
            keptClusters.reserve(this->pcClustersVisual_.size());
            keptCenters.reserve(this->pcClusterCentersVisual_.size());
            keptStds.reserve(this->pcClusterStdsVisual_.size());

            for (size_t i = 0; i < this->dbBBoxes_.size(); ++i) {
                if (this->isOversizedBBox(this->dbBBoxes_[i])) {
                    continue;
                }
                keptBBoxes.push_back(this->dbBBoxes_[i]);
                if (i < this->pcClustersVisual_.size()) keptClusters.push_back(this->pcClustersVisual_[i]);
                if (i < this->pcClusterCentersVisual_.size()) keptCenters.push_back(this->pcClusterCentersVisual_[i]);
                if (i < this->pcClusterStdsVisual_.size()) keptStds.push_back(this->pcClusterStdsVisual_[i]);
            }

            this->dbBBoxes_ = std::move(keptBBoxes);
            this->pcClustersVisual_ = std::move(keptClusters);
            this->pcClusterCentersVisual_ = std::move(keptCenters);
            this->pcClusterStdsVisual_ = std::move(keptStds);
        }
    }


    void dynamicDetector::lidarDetect(){
        if (this->lidarDetector_ == nullptr){
            this->lidarDetector_.reset(new lidarDetector());
        }

        // aggiorna sempre i parametri, anche se il detector esiste già
        this->lidarDetector_->setParams(
            this->lidarDBEpsilon_,
            this->lidarDBMinPoints_,
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

        if (this->lidarCloud_ != nullptr){
            this->lidarDetector_->getPointcloud(this->lidarCloud_);
            this->lidarDetector_->lidarDBSCAN();

            std::vector<joDetector::Cluster> lidarClustersRaw = this->lidarDetector_->getClusters();
            std::vector<joDetector::Cluster> lidarClustersFiltered;
            std::vector<joDetector::box3D> lidarBBoxesRaw = this->lidarDetector_->getBBoxes();
            std::vector<joDetector::box3D> lidarBBoxesFiltered;

            for (size_t i = 0; i < lidarBBoxesRaw.size(); ++i){
                const joDetector::box3D& lidarBBox = lidarBBoxesRaw[i];

                if (lidarBBox.x_width > this->maxObjectSize_(0) ||
                    lidarBBox.y_width > this->maxObjectSize_(1) ||
                    lidarBBox.z_width > this->maxObjectSize_(2) ||
                    this->isOversizedBBox(lidarBBox)){
                    continue;
                }

                lidarBBoxesFiltered.push_back(lidarBBox);
                lidarClustersFiltered.push_back(lidarClustersRaw[i]);
            }

            this->lidarBBoxes_ = lidarBBoxesFiltered;
            this->lidarClusters_ = lidarClustersFiltered;
        }
    }

    void dynamicDetector::BboxesMerger(
                                    const std::vector<joDetector::box3D>& group1BBoxes_,
                                    const std::vector<joDetector::box3D>& group2BBoxes_,

                                    const std::vector<std::vector<Eigen::Vector3d>>& group1pcClusters_,
                                    const std::vector<Eigen::Vector3d>& group1pcClusterCenters_,
                                    const std::vector<Eigen::Vector3d>& group1pcClusterStds_,

                                    const std::vector<std::vector<Eigen::Vector3d>>& group2pcClusters_,
                                    const std::vector<Eigen::Vector3d>& group2pcClusterCenters_,
                                    const std::vector<Eigen::Vector3d>& group2pcClusterStds_,

                                    std::vector<joDetector::box3D>& BBoxesTemp,
                                    std::vector<std::vector<Eigen::Vector3d>>& PcClustersTemp,
                                    std::vector<Eigen::Vector3d>& PcClusterCentersTemp,
                                    std::vector<Eigen::Vector3d>& PcClusterStdsTemp,

                                    std::string merging_style,
                                    bool flag_group1,
                                    bool flag_group2,
                                    double boxIOUThresh_,
                                    double boxIOVThresh_,
                                    bool leaf_only)
    {
        size_t M = group1BBoxes_.size();
        size_t N = group2BBoxes_.size();
        if (M == 0 || N == 0) {
            if (N == 0 && flag_group1) {
                for (size_t i = 0; i < M; ++i) {
                    BBoxesTemp.push_back(group1BBoxes_[i]);
                    PcClustersTemp.push_back(group1pcClusters_[i]);
                    PcClusterCentersTemp.push_back(group1pcClusterCenters_[i]);
                    PcClusterStdsTemp.push_back(group1pcClusterStds_[i]);
                }
            }
            if (M == 0 && flag_group2) {
                for (size_t j = 0; j < N; ++j) {
                    BBoxesTemp.push_back(group2BBoxes_[j]);
                    PcClustersTemp.push_back(group2pcClusters_[j]);
                    PcClusterCentersTemp.push_back(group2pcClusterCenters_[j]);
                    PcClusterStdsTemp.push_back(group2pcClusterStds_[j]);
                }
            }
            return;
        }
        
        Eigen::MatrixXd IOU = Eigen::MatrixXd::Zero(M, N);
        Eigen::MatrixXd IOV_g1 = Eigen::MatrixXd::Zero(M, N);
        Eigen::MatrixXd IOV_g2 = Eigen::MatrixXd::Zero(M, N);

        std::vector<bool> usedGroup1(M, false);
        std::vector<bool> usedGroup2(N, false);

        double eps = 1e-6;
        std::vector<joDetector::Edge> edges;

        // =========================
        // STEP 0: MATRICES
        // =========================
        for (size_t i = 0; i < M; ++i)
        {
            for (size_t j = 0; j < N; ++j)
            {       
                IOU(i,j) = this->calBoxIOU(group1BBoxes_[i], group2BBoxes_[j]);
                IOV_g1(i,j) = this->calBoxIOV(group1BBoxes_[i], group2BBoxes_[j]);
                IOV_g2(i,j) = this->calBoxIOV(group2BBoxes_[j], group1BBoxes_[i]);
            }
        }

        // =========================
        // STEP 1: MUTUAL IOU MATCH
        // =========================
        for (size_t i = 0; i < M; ++i)
        {
            if (usedGroup1[i]) continue;

            double bestIOU = -1;
            int best_j = -1;

            for (size_t j = 0; j < N; ++j)
            {
                if (usedGroup2[j]) continue;

                if (IOU(i,j) > bestIOU)
                {
                    bestIOU = IOU(i,j);
                    best_j = j;
                }
            }

            if (best_j == -1) continue;

            double bestIOU_back = -1;
            int best_i = -1;

            for (size_t k = 0; k < M; ++k)
            {
                if (usedGroup1[k]) continue;

                if (IOU(k, best_j) > bestIOU_back)
                {
                    bestIOU_back = IOU(k, best_j);
                    best_i = k;
                }
            }

            if (best_i == (int)i &&
                bestIOU > boxIOUThresh_ &&
                bestIOU_back > boxIOUThresh_)
            {
                auto b1 = group1BBoxes_[i];
                auto b2 = group2BBoxes_[best_j];

                double xmax = std::max(b1.x+b1.x_width/2, b2.x+b2.x_width/2);
                double xmin = std::min(b1.x-b1.x_width/2, b2.x-b2.x_width/2);
                double ymax = std::max(b1.y+b1.y_width/2, b2.y+b2.y_width/2);
                double ymin = std::min(b1.y-b1.y_width/2, b2.y-b2.y_width/2);
                double zmax = std::max(b1.z+b1.z_width/2, b2.z+b2.z_width/2);
                double zmin = std::min(b1.z-b1.z_width/2, b2.z-b2.z_width/2);

                joDetector::box3D bbox;
                bbox.x = (xmin+xmax)/2;
                bbox.y = (ymin+ymax)/2;
                bbox.z = (zmin+zmax)/2;
                bbox.x_width = xmax-xmin;
                bbox.y_width = ymax-ymin;
                bbox.z_width = zmax-zmin;
                bbox.Vx = 0;
                bbox.Vy = 0;

                std::vector<Eigen::Vector3d> cluster;
                cluster.insert(cluster.end(),
                    group1pcClusters_[i].begin(),
                    group1pcClusters_[i].end());
                cluster.insert(cluster.end(),
                    group2pcClusters_[best_j].begin(),
                    group2pcClusters_[best_j].end());

                Eigen::Vector3d center = Eigen::Vector3d::Zero();
                for (const auto& p : cluster) {
                    center += p;
                }
                if (!cluster.empty()) {
                    center /= static_cast<double>(cluster.size());
                }

                Eigen::Vector3d stddev = Eigen::Vector3d::Zero();
                for (const auto& p : cluster) {
                    stddev += (p - center).cwiseAbs2();
                }
                if (!cluster.empty()) {
                    stddev = (stddev / static_cast<double>(cluster.size())).cwiseSqrt();
                }

                BBoxesTemp.push_back(bbox);
                PcClustersTemp.push_back(cluster);
                PcClusterCentersTemp.push_back(center);
                PcClusterStdsTemp.push_back(stddev);

                usedGroup1[i] = true;
                usedGroup2[best_j] = true;
            }
        }

        // =========================
        // STEP 2: BUILD GRAPH (IOV)
        // =========================
        for (size_t i = 0; i < M; ++i)
        {
            if (usedGroup1[i]) continue;

            for (size_t j = 0; j < N; ++j)
            {
                if (usedGroup2[j]) continue;

                double iov1 = IOV_g1(i,j);
                double iov2 = IOV_g2(i,j);

                if (iov1 < eps && iov2 < eps) continue;

                joDetector::Edge e;

                if (iov1 > iov2)
                {
                    e.parent = { (int)j, false };
                    e.child  = { (int)i, true };
                    e.weight = iov1;
                }
                else
                {
                    e.parent = { (int)i, true };
                    e.child  = { (int)j, false };
                    e.weight = iov2;
                }

                if (e.weight >= boxIOVThresh_)
                    edges.push_back(e);
            }
        }

        // =========================
        // STEP 3: BEST PARENT
        // =========================
        std::map<std::pair<int,bool>, joDetector::Edge> bestParent;

        for (const auto& e : edges)
        {
            auto key = std::make_pair(e.child.idx, e.child.is_group1);

            if (bestParent.find(key) == bestParent.end() ||
                e.weight > bestParent[key].weight)
            {
                bestParent[key] = e;
            }
        }

        // =========================
        // STEP 4: GRAPH
        // =========================
        std::map<std::pair<int,bool>, std::vector<joDetector::Node>> childrenMap;
        std::set<std::pair<int,bool>> allNodes, childNodes;

        for (const auto& kv : bestParent)
        {
            const auto& e = kv.second;

            auto p = std::make_pair(e.parent.idx, e.parent.is_group1);
            auto c = std::make_pair(e.child.idx, e.child.is_group1);

            childrenMap[p].push_back(e.child);

            allNodes.insert(p);
            allNodes.insert(c);
            childNodes.insert(c);
        }
        // =========================
        // STEP X: MARK ALL GRAPH NODES AS USED
        // =========================
        for (const auto& k : allNodes)
        {
            if (k.second)  // is_group1 == true
                usedGroup1[k.first] = true;
            else           // group2
                usedGroup2[k.first] = true;
        }
        // =========================
        // 🔥 FIX CRITICO: SMALLER = LEAF ONLY
        // ==========================
        if (merging_style == "smaller")
        {
            for (const auto& k : allNodes)
            {
                // skip non-leaf
                if (childrenMap.find(k) != childrenMap.end()) continue;

                joDetector::Node leaf{k.first, k.second};

                // =========================
                // 🔥 TROVA IL PARENT (se esiste)
                // =========================
                bool hasParent = false;
                joDetector::Node parentNode;

                for (const auto& kv : bestParent)
                {
                    if (kv.first.first == leaf.idx &&
                        kv.first.second == leaf.is_group1)
                    {
                        parentNode = kv.second.parent;
                        hasParent = true;
                        break;
                    }
                }

                // =========================
                // 🔥 CASO CONSERVATIVO: 1 parent - 1 child (skipped when leaf_only)
                // =========================
                if (hasParent && !leaf_only)
                {
                    auto parentKey = std::make_pair(parentNode.idx, parentNode.is_group1);

                    // parent ha SOLO questo figlio?
                    if (childrenMap[parentKey].size() == 1)
                    {
                        // 👉 MERGE padre + figlio
                        joDetector::box3D b1, b2;
                        std::vector<Eigen::Vector3d> cluster;

                        // leaf
                        if (leaf.is_group1)
                        {
                            b1 = group1BBoxes_[leaf.idx];
                            cluster.insert(cluster.end(),
                                group1pcClusters_[leaf.idx].begin(),
                                group1pcClusters_[leaf.idx].end());
                        }
                        else
                        {
                            b1 = group2BBoxes_[leaf.idx];
                            cluster.insert(cluster.end(),
                                group2pcClusters_[leaf.idx].begin(),
                                group2pcClusters_[leaf.idx].end());
                        }

                        // parent
                        if (parentNode.is_group1)
                        {
                            b2 = group1BBoxes_[parentNode.idx];
                            cluster.insert(cluster.end(),
                                group1pcClusters_[parentNode.idx].begin(),
                                group1pcClusters_[parentNode.idx].end());
                        }
                        else
                        {
                            b2 = group2BBoxes_[parentNode.idx];
                            cluster.insert(cluster.end(),
                                group2pcClusters_[parentNode.idx].begin(),
                                group2pcClusters_[parentNode.idx].end());
                        }

                        double xmax = std::max(b1.x+b1.x_width/2, b2.x+b2.x_width/2);
                        double xmin = std::min(b1.x-b1.x_width/2, b2.x-b2.x_width/2);
                        double ymax = std::max(b1.y+b1.y_width/2, b2.y+b2.y_width/2);
                        double ymin = std::min(b1.y-b1.y_width/2, b2.y-b2.y_width/2);
                        double zmax = std::max(b1.z+b1.z_width/2, b2.z+b2.z_width/2);
                        double zmin = std::min(b1.z-b1.z_width/2, b2.z-b2.z_width/2);

                        joDetector::box3D bbox;
                        bbox.x = (xmin+xmax)/2;
                        bbox.y = (ymin+ymax)/2;
                        bbox.z = (zmin+zmax)/2;
                        bbox.x_width = xmax-xmin;
                        bbox.y_width = ymax-ymin;
                        bbox.z_width = zmax-zmin;
                        bbox.Vx = 0;
                        bbox.Vy = 0;

                        // center + std
                        Eigen::Vector3d center = Eigen::Vector3d::Zero();
                        for (auto& p : cluster) center += p;
                        if (!cluster.empty()) center /= cluster.size();

                        Eigen::Vector3d stddev = Eigen::Vector3d::Zero();
                        for (auto& p : cluster)
                            stddev += (p - center).cwiseAbs2();
                        if (!cluster.empty())
                            stddev = (stddev / cluster.size()).cwiseSqrt();

                        BBoxesTemp.push_back(bbox);
                        PcClustersTemp.push_back(cluster);
                        PcClusterCentersTemp.push_back(center);
                        PcClusterStdsTemp.push_back(stddev);

                        continue;
                    }
                }

                // =========================
                // DEFAULT: SOLO LEAF
                // =========================
                joDetector::box3D bbox;
                std::vector<Eigen::Vector3d> cluster;
                Eigen::Vector3d center, stddev;

                if (leaf.is_group1)
                {
                    bbox = group1BBoxes_[leaf.idx];
                    cluster = group1pcClusters_[leaf.idx];
                    center  = group1pcClusterCenters_[leaf.idx];
                    stddev  = group1pcClusterStds_[leaf.idx];
                }
                else
                {
                    bbox = group2BBoxes_[leaf.idx];
                    cluster = group2pcClusters_[leaf.idx];
                    center  = group2pcClusterCenters_[leaf.idx];
                    stddev  = group2pcClusterStds_[leaf.idx];
                }

                BBoxesTemp.push_back(bbox);
                PcClustersTemp.push_back(cluster);
                PcClusterCentersTemp.push_back(center);
                PcClusterStdsTemp.push_back(stddev);
            }
        }
        else if (merging_style == "bigger")
        {
            // DFS per visitare il sottoalbero
            std::function<void(const joDetector::Node&, std::vector<joDetector::Node>&)> dfs =
            [&](const joDetector::Node& n, std::vector<joDetector::Node>& out)
            {
                out.push_back(n);

                auto key = std::make_pair(n.idx, n.is_group1);
                if (childrenMap.find(key) == childrenMap.end()) return;

                for (const auto& c : childrenMap[key])
                    dfs(c, out);
            };

            // trova le root (nodi che non sono figli)
            std::set<std::pair<int,bool>> childNodes;
            for (const auto& kv : childrenMap)
            {
                for (const auto& c : kv.second)
                    childNodes.insert({c.idx, c.is_group1});
            }

            std::vector<joDetector::Node> roots;
            for (const auto& k : allNodes)
            {
                if (childNodes.find(k) == childNodes.end())
                    roots.push_back({k.first, k.second});
            }

            // MERGE PER OGNI ALBERO
            for (const auto& root : roots)
            {
                std::vector<joDetector::Node> subtree;
                dfs(root, subtree);

                double xmin =  std::numeric_limits<double>::max();
                double ymin =  std::numeric_limits<double>::max();
                double zmin =  std::numeric_limits<double>::max();

                double xmax = -std::numeric_limits<double>::max();
                double ymax = -std::numeric_limits<double>::max();
                double zmax = -std::numeric_limits<double>::max();

                std::vector<Eigen::Vector3d> mergedCluster;

                for (const auto& n : subtree)
                {
                    joDetector::box3D b;

                    // 👉 SCELTA CORRETTA DELLA SORGENTE (FIX PRINCIPALE)
                    if (n.is_group1)
                    {
                        b = group1BBoxes_[n.idx];
                        mergedCluster.insert(mergedCluster.end(),
                                            group1pcClusters_[n.idx].begin(),
                                            group1pcClusters_[n.idx].end());
                    }
                    else
                    {
                        b = group2BBoxes_[n.idx];
                        mergedCluster.insert(mergedCluster.end(),
                                            group2pcClusters_[n.idx].begin(),
                                            group2pcClusters_[n.idx].end());
                    }

                    xmax = std::max(xmax, b.x + b.x_width/2);
                    xmin = std::min(xmin, b.x - b.x_width/2);
                    ymax = std::max(ymax, b.y + b.y_width/2);
                    ymin = std::min(ymin, b.y - b.y_width/2);
                    zmax = std::max(zmax, b.z + b.z_width/2);
                    zmin = std::min(zmin, b.z - b.z_width/2);
                }

                // bounding box finale
                joDetector::box3D bbox;
                bbox.x = (xmin+xmax)/2;
                bbox.y = (ymin+ymax)/2;
                bbox.z = (zmin+zmax)/2;
                bbox.x_width = xmax-xmin;
                bbox.y_width = ymax-ymin;
                bbox.z_width = zmax-zmin;
                bbox.Vx = 0;
                bbox.Vy = 0;

                // calcolo feature cluster
                Eigen::Vector3d center = Eigen::Vector3d::Zero();
                for (const auto& p : mergedCluster) center += p;
                if (!mergedCluster.empty()) center /= mergedCluster.size();

                Eigen::Vector3d stddev = Eigen::Vector3d::Zero();
                for (const auto& p : mergedCluster)
                    stddev += (p - center).cwiseAbs2();
                if (!mergedCluster.empty())
                    stddev = (stddev / mergedCluster.size()).cwiseSqrt();

                BBoxesTemp.push_back(bbox);
                PcClustersTemp.push_back(mergedCluster);
                PcClusterCentersTemp.push_back(center);
                PcClusterStdsTemp.push_back(stddev);
            }
        }

        // =========================
        // ADD UNUSED
        // =========================
        if (flag_group1)
        {
            for (size_t i = 0; i < M; ++i)
            {
                if (!usedGroup1[i])
                {
                    BBoxesTemp.push_back(group1BBoxes_[i]);
                    PcClustersTemp.push_back(group1pcClusters_[i]);
                    PcClusterCentersTemp.push_back(group1pcClusterCenters_[i]);
                    PcClusterStdsTemp.push_back(group1pcClusterStds_[i]);
                }
            }
        }

        if (flag_group2)
        {
            for (size_t j = 0; j < N; ++j)
            {
                if (!usedGroup2[j])
                {
                    BBoxesTemp.push_back(group2BBoxes_[j]);
                    PcClustersTemp.push_back(group2pcClusters_[j]);
                    PcClusterCentersTemp.push_back(group2pcClusterCenters_[j]);
                    PcClusterStdsTemp.push_back(group2pcClusterStds_[j]);
                }
            }
        }
    }

    void dynamicDetector::mergeNestedGroup(
        const std::vector<joDetector::box3D>& inBoxes,
        const std::vector<std::vector<Eigen::Vector3d>>& inClusters,
        const std::vector<Eigen::Vector3d>& inCenters,
        const std::vector<Eigen::Vector3d>& inStds,

        std::vector<joDetector::box3D>& outBoxes,
        std::vector<std::vector<Eigen::Vector3d>>& outClusters,
        std::vector<Eigen::Vector3d>& outCenters,
        std::vector<Eigen::Vector3d>& outStds)
    {
        size_t N = inBoxes.size();
        // std::cout << "[DEBUG] mergeNestedGroup: N=" << N
        //           << ", inClusters.size()=" << inClusters.size()
        //           << ", inCenters.size()=" << inCenters.size()
        //           << ", inStds.size()=" << inStds.size() << std::endl;
        if (N == 0) return;

        // Guard: if cluster arrays don't match box count, pad with empties
        // This prevents out-of-bounds access when uvBBoxes_ uses pcClustersVisual_
        std::vector<std::vector<Eigen::Vector3d>> safeClusters = inClusters;
        std::vector<Eigen::Vector3d> safeCenters = inCenters;
        std::vector<Eigen::Vector3d> safeStds = inStds;
        if (safeClusters.size() < N) {
            std::cout << "[WARN] mergeNestedGroup: inClusters.size()=" << inClusters.size() << " < N=" << N << ", padding with empties" << std::endl;
            safeClusters.resize(N);
        }
        if (safeCenters.size() < N) {
            std::cout << "[WARN] mergeNestedGroup: inCenters.size()=" << inCenters.size() << " < N=" << N << ", padding with empties" << std::endl;
            safeCenters.resize(N, Eigen::Vector3d::Zero());
        }
        if (safeStds.size() < N) {
            std::cout << "[WARN] mergeNestedGroup: inStds.size()=" << inStds.size() << " < N=" << N << ", padding with empties" << std::endl;
            safeStds.resize(N, Eigen::Vector3d::Zero());
        }

        double iouThresh = this->samegroupIOUThresh_;
        double iovThresh = this->samegroupIOVThresh_;

        Eigen::MatrixXd IOU = Eigen::MatrixXd::Zero(N, N);

        // =========================
        // STEP 0: IOU MATRIX
        // =========================
        for (size_t i = 0; i < N; ++i)
        {
            for (size_t j = 0; j < N; ++j)
            {
                if (i == j) continue;
                IOU(i,j) = this->calBoxIOU(inBoxes[i], inBoxes[j]);
            }
        }

        std::vector<bool> used(N, false);

        // =========================
        // STEP 1: MUTUAL IOU MATCH
        // =========================
        for (size_t i = 0; i < N; ++i)
        {
            if (used[i]) continue;

            int best_j = -1;
            double best_iou = -1;

            for (size_t j = 0; j < N; ++j)
            {
                if (i == j || used[j]) continue;

                if (IOU(i,j) > best_iou)
                {
                    best_iou = IOU(i,j);
                    best_j = (int)j;
                }
            }

            if (best_j == -1) continue;

            int best_i_back = -1;
            double best_iou_back = -1;

            for (size_t k = 0; k < N; ++k)
            {
                if ((int)k == best_j || used[k]) continue;

                if (IOU(k, best_j) > best_iou_back)
                {
                    best_iou_back = IOU(k, best_j);
                    best_i_back = (int)k;
                }
            }

            if (best_i_back == (int)i && best_iou > iouThresh && best_iou_back > iouThresh)
            {
                std::vector<int> comp = { (int)i, best_j };

                joDetector::box3D box;
                std::vector<Eigen::Vector3d> cluster;
                Eigen::Vector3d center, stddev;

                mergeBoxesSet(inBoxes, safeClusters, comp,
                            box, cluster, center, stddev);

                outBoxes.push_back(box);
                outClusters.push_back(cluster);
                outCenters.push_back(center);
                outStds.push_back(stddev);

                used[i] = true;
                used[best_j] = true;
            }
        }

        // =========================
        // STEP 2: BUILD EDGES (NESTED via IOV)
        // =========================
        struct Edge { int parent; int child; double w; };
        std::vector<Edge> edges;

        for (size_t i = 0; i < N; ++i)
        {
            if (used[i]) continue;

            for (size_t j = 0; j < N; ++j)
            {
                if (i == j || used[j]) continue;

                double iov_ij = this->calBoxIOV(inBoxes[i], inBoxes[j]);
                double iov_ji = this->calBoxIOV(inBoxes[j], inBoxes[i]);

                // 🔥 nested detection (asimmetrico!)
                if (iov_ij > iovThresh || iov_ji > iovThresh)
                {
                    Edge e;

                    if (iov_ij > iov_ji)
                    {
                        // j contiene i
                        e.parent = (int)j;
                        e.child  = (int)i;
                        e.w = iov_ij;
                    }
                    else
                    {
                        // i contiene j
                        e.parent = (int)i;
                        e.child  = (int)j;
                        e.w = iov_ji;
                    }

                    edges.push_back(e);
                }
            }
        }

        // =========================
        // STEP 3: KEEP BEST PARENT
        // =========================
        std::map<int, Edge> bestParent;

        for (const auto& e : edges)
        {
            if (bestParent.find(e.child) == bestParent.end() ||
                e.w > bestParent[e.child].w)
            {
                bestParent[e.child] = e;
            }
        }

        // =========================
        // STEP 4: BUILD GRAPH
        // =========================
        std::map<int, std::vector<int>> children;
        std::set<int> allNodes, childNodes;

        for (const auto& kv : bestParent)
        {
            const auto& e = kv.second;

            children[e.parent].push_back(e.child);

            allNodes.insert(e.parent);
            allNodes.insert(e.child);
            childNodes.insert(e.child);
        }

        // =========================
        // STEP 5: MERGE TREES
        // =========================
        for (auto root : allNodes)
        {
            if (childNodes.count(root)) continue;

            std::vector<int> subtree;
            std::queue<int> q;
            q.push(root);

            while (!q.empty())
            {
                int u = q.front(); q.pop();
                subtree.push_back(u);

                for (auto c : children[u])
                    q.push(c);
            }

            joDetector::box3D box;
            std::vector<Eigen::Vector3d> cluster;
            Eigen::Vector3d center, stddev;

            mergeBoxesSet(inBoxes, safeClusters, subtree,
                        box, cluster, center, stddev);

            outBoxes.push_back(box);
            outClusters.push_back(cluster);
            outCenters.push_back(center);
            outStds.push_back(stddev);

            for (auto idx : subtree)
                used[idx] = true;
        }

        // =========================
        // STEP 6: LEFTOVER
        // =========================
        for (size_t i = 0; i < N; ++i)
        {
            if (!used[i])
            {
                outBoxes.push_back(inBoxes[i]);
                outClusters.push_back(safeClusters[i]);
                outCenters.push_back(safeCenters[i]);
                outStds.push_back(safeStds[i]);
            }
        }
    }

    void dynamicDetector::mergeBoxesSet(
        const std::vector<joDetector::box3D>& boxes,
        const std::vector<std::vector<Eigen::Vector3d>>& clusters,
        const std::vector<int>& indices,
        joDetector::box3D& outBox,
        std::vector<Eigen::Vector3d>& outCluster,
        Eigen::Vector3d& center,
        Eigen::Vector3d& stddev)
    {
        double xmin =  std::numeric_limits<double>::max();
        double ymin =  std::numeric_limits<double>::max();
        double zmin =  std::numeric_limits<double>::max();

        double xmax = -std::numeric_limits<double>::max();
        double ymax = -std::numeric_limits<double>::max();
        double zmax = -std::numeric_limits<double>::max();

        for (int idx : indices)
        {
            const auto& b = boxes[idx];

            xmax = std::max(xmax, b.x + b.x_width/2);
            xmin = std::min(xmin, b.x - b.x_width/2);
            ymax = std::max(ymax, b.y + b.y_width/2);
            ymin = std::min(ymin, b.y - b.y_width/2);
            zmax = std::max(zmax, b.z + b.z_width/2);
            zmin = std::min(zmin, b.z - b.z_width/2);

            outCluster.insert(outCluster.end(),
                            clusters[idx].begin(),
                            clusters[idx].end());
        }

        outBox.x = (xmin+xmax)/2;
        outBox.y = (ymin+ymax)/2;
        outBox.z = (zmin+zmax)/2;
        outBox.x_width = xmax-xmin;
        outBox.y_width = ymax-ymin;
        outBox.z_width = zmax-zmin;
        outBox.Vx = 0;
        outBox.Vy = 0;

        // center
        center = Eigen::Vector3d::Zero();
        for (auto& p : outCluster) center += p;
        if (!outCluster.empty()) center /= outCluster.size();

        // std
        stddev = Eigen::Vector3d::Zero();
        for (auto& p : outCluster)
            stddev += (p - center).cwiseAbs2();
        if (!outCluster.empty())
            stddev = (stddev / outCluster.size()).cwiseSqrt();
    }

    void dynamicDetector::filterLVBBoxes(){
        // std::cout << "[DEBUG] filterLVBBoxes: START" << std::endl;
        // std::cout << "[DEBUG] filterLVBBoxes: uvBBoxes_ size = " << this->uvBBoxes_.size()
        //           << ", dbBBoxes_ size = " << this->dbBBoxes_.size()
        //           << ", pcClustersVisual_ size = " << this->pcClustersVisual_.size()
        //           << ", pcClusterCentersVisual_ size = " << this->pcClusterCentersVisual_.size()
        //           << ", pcClusterStdsVisual_ size = " << this->pcClusterStdsVisual_.size()
        //           << ", lidarBBoxes_ size = " << this->lidarBBoxes_.size() << std::endl;

        std::vector<joDetector::box3D> filteredBBoxesTemp;
        std::vector<std::vector<Eigen::Vector3d>> filteredPcClustersTemp;
        std::vector<Eigen::Vector3d> filteredPcClusterCentersTemp;
        std::vector<Eigen::Vector3d> filteredPcClusterStdsTemp; 

        std::vector<joDetector::box3D> visualBBoxesTemp;
        std::vector<std::vector<Eigen::Vector3d>> visualPcClustersTemp;
        std::vector<Eigen::Vector3d> visualPcClusterCentersTemp;
        std::vector<Eigen::Vector3d> visualPcClusterStdsTemp; // store visual output

        std::vector<joDetector::box3D> lidarBBoxesTemp;
        std::vector<std::vector<Eigen::Vector3d>> lidarPcClustersTemp;
        std::vector<Eigen::Vector3d> lidarPcClusterCentersTemp;
        std::vector<Eigen::Vector3d> lidarPcClusterStdsTemp; // store lidar output

        auto removeOversizedBoxes = [this](
            std::vector<joDetector::box3D>& boxes,
            std::vector<std::vector<Eigen::Vector3d>>& clusters,
            std::vector<Eigen::Vector3d>& centers,
            std::vector<Eigen::Vector3d>& stds)
        {
            std::vector<joDetector::box3D> keptBoxes;
            std::vector<std::vector<Eigen::Vector3d>> keptClusters;
            std::vector<Eigen::Vector3d> keptCenters;
            std::vector<Eigen::Vector3d> keptStds;
            keptBoxes.reserve(boxes.size());
            keptClusters.reserve(clusters.size());
            keptCenters.reserve(centers.size());
            keptStds.reserve(stds.size());

            for (size_t i = 0; i < boxes.size(); ++i) {
                if (this->isOversizedBBox(boxes[i])) {
                    continue;
                }
                keptBoxes.push_back(boxes[i]);
                if (i < clusters.size()) keptClusters.push_back(clusters[i]);
                if (i < centers.size()) keptCenters.push_back(centers[i]);
                if (i < stds.size()) keptStds.push_back(stds[i]);
            }

            boxes = std::move(keptBoxes);
            clusters = std::move(keptClusters);
            centers = std::move(keptCenters);
            stds = std::move(keptStds);
        };

        // =========================
        // STEP 0: MERGE NESTED UV + DB (INTRA-GROUP)
        // =========================
        std::vector<joDetector::box3D> uvBBoxesFiltered;
        std::vector<std::vector<Eigen::Vector3d>> uvPcClustersFiltered;
        std::vector<Eigen::Vector3d> uvPcClusterCentersFiltered;
        std::vector<Eigen::Vector3d> uvPcClusterStdsFiltered;

        std::vector<joDetector::box3D> dbBBoxesFiltered;
        std::vector<std::vector<Eigen::Vector3d>> dbPcClustersFiltered;
        std::vector<Eigen::Vector3d> dbPcClusterCentersFiltered;
        std::vector<Eigen::Vector3d> dbPcClusterStdsFiltered;

        // UV: extract depth points falling inside each UV bbox (world frame)
        // UV bboxes have no associated pointcloud from the uv-map detector,
        // so we build their clusters directly from filteredDepthPoints_.
        std::vector<std::vector<Eigen::Vector3d>> uvPcClusters(this->uvBBoxes_.size());
        std::vector<Eigen::Vector3d> uvPcCenters(this->uvBBoxes_.size(), Eigen::Vector3d::Zero());
        std::vector<Eigen::Vector3d> uvPcStds(this->uvBBoxes_.size(), Eigen::Vector3d::Zero());

        for (size_t i = 0; i < this->uvBBoxes_.size(); ++i) {
            const auto& b = this->uvBBoxes_[i];
            const double hx = b.x_width / 2.0;
            const double hy = b.y_width / 2.0;
            const double hz = b.z_width / 2.0;
            for (const auto& p : this->filteredDepthPoints_) {
                if (std::abs(p.x() - b.x) <= hx &&
                    std::abs(p.y() - b.y) <= hy &&
                    std::abs(p.z() - b.z) <= hz) {
                    uvPcClusters[i].push_back(p);
                }
            }
            if (!uvPcClusters[i].empty()) {
                Eigen::Vector3d center = Eigen::Vector3d::Zero();
                for (const auto& p : uvPcClusters[i]) center += p;
                center /= static_cast<double>(uvPcClusters[i].size());
                uvPcCenters[i] = center;
                Eigen::Vector3d stddev = Eigen::Vector3d::Zero();
                for (const auto& p : uvPcClusters[i])
                    stddev += (p - center).cwiseAbs2();
                uvPcStds[i] = (stddev / static_cast<double>(uvPcClusters[i].size())).cwiseSqrt();
            }
        }

        // std::cout << "[DEBUG] filterLVBBoxes: calling mergeNestedGroup for UV" << std::endl;
        this->mergeNestedGroup(
            this->uvBBoxes_,
            uvPcClusters,
            uvPcCenters,
            uvPcStds,

            uvBBoxesFiltered,
            uvPcClustersFiltered,
            uvPcClusterCentersFiltered,
            uvPcClusterStdsFiltered
        );
        // std::cout << "[DEBUG] filterLVBBoxes: after UV mergeNestedGroup, uvBBoxesFiltered size = " << uvBBoxesFiltered.size() << std::endl;

        // DBSCAN
        // std::cout << "[DEBUG] filterLVBBoxes: calling mergeNestedGroup for DBSCAN" << std::endl;
        this->mergeNestedGroup(
            this->dbBBoxes_,
            this->pcClustersVisual_,
            this->pcClusterCentersVisual_,
            this->pcClusterStdsVisual_,

            dbBBoxesFiltered,
            dbPcClustersFiltered,
            dbPcClusterCentersFiltered,
            dbPcClusterStdsFiltered
        );
        // std::cout << "[DEBUG] filterLVBBoxes: after DBSCAN mergeNestedGroup, dbBBoxesFiltered size = " << dbBBoxesFiltered.size() << std::endl;

        removeOversizedBoxes(uvBBoxesFiltered,
                               uvPcClustersFiltered,
                               uvPcClusterCentersFiltered,
                               uvPcClusterStdsFiltered);
        removeOversizedBoxes(dbBBoxesFiltered,
                               dbPcClustersFiltered,
                               dbPcClusterCentersFiltered,
                               dbPcClusterStdsFiltered);
        this->publish3dBox(uvBBoxesFiltered, this->uvBBoxesFilteredPub_, 0, 1, 0.5);
        this->publish3dBox(dbBBoxesFiltered, this->dbBBoxesFilteredPub_, 1, 0.5, 0);

        // STEP 1: Merge UV and DBSCAN bounding boxes to get visual bounding boxes (with clusters and features)
        this->BboxesMerger(
            uvBBoxesFiltered, 
            dbBBoxesFiltered,

            uvPcClustersFiltered,
            uvPcClusterCentersFiltered,
            uvPcClusterStdsFiltered,

            dbPcClustersFiltered,
            dbPcClusterCentersFiltered,
            dbPcClusterStdsFiltered,

            visualBBoxesTemp, 
            visualPcClustersTemp, 
            visualPcClusterCentersTemp, 
            visualPcClusterStdsTemp,

            this->visualmergingFlag_, 
            this->uvUnmergedFlag_, 
            this->dbUnmergedFlag_,

            this->visualboxIOUThresh_, 
            this->visualboxIOVThresh_);
        
        removeOversizedBoxes(visualBBoxesTemp,
                               visualPcClustersTemp,
                               visualPcClusterCentersTemp,
                               visualPcClusterStdsTemp);
        this->visualBBoxes_ = visualBBoxesTemp;

        // STEP 2: Get lidar bboxes and its corresponding clusters and features
        // lidar bbox filter
        for (size_t i = 0; i < this->lidarBBoxes_.size(); ++i) {
            joDetector::box3D lidarBBox = this->lidarBBoxes_[i];
            
            // get corresponding point cloud cluster
            joDetector::Cluster cluster = this->lidarClusters_[i];

            std::vector<Eigen::Vector3d> pcCluster;
            for (const pcl::PointXYZ& point : cluster.points->points) {
                pcCluster.emplace_back(point.x, point.y, point.z);
            }

            // extract the cluster center
            Eigen::Vector3d clusterCenter(cluster.centroid[0], cluster.centroid[1], cluster.centroid[2]);

            // compute std
            Eigen::Vector3d clusterStd = cluster.eigen_values.cwiseSqrt().cast<double>();

            lidarBBoxesTemp.push_back(lidarBBox);
            lidarPcClustersTemp.push_back(pcCluster);
            lidarPcClusterCentersTemp.push_back(clusterCenter);
            lidarPcClusterStdsTemp.push_back(clusterStd);
        }

        // =========================
        // STEP 2.5: MERGE NESTED LIDAR
        // =========================
        std::vector<joDetector::box3D> lidarBBoxesFiltered;
        std::vector<std::vector<Eigen::Vector3d>> lidarPcClustersFiltered;
        std::vector<Eigen::Vector3d> lidarPcClusterCentersFiltered;
        std::vector<Eigen::Vector3d> lidarPcClusterStdsFiltered;

        this->mergeNestedGroup(
            lidarBBoxesTemp,
            lidarPcClustersTemp,
            lidarPcClusterCentersTemp,
            lidarPcClusterStdsTemp,

            lidarBBoxesFiltered,
            lidarPcClustersFiltered,
            lidarPcClusterCentersFiltered,
            lidarPcClusterStdsFiltered
        );

        removeOversizedBoxes(lidarBBoxesFiltered,
                               lidarPcClustersFiltered,
                               lidarPcClusterCentersFiltered,
                               lidarPcClusterStdsFiltered);
        this->publish3dBox(lidarBBoxesFiltered, this->lidarBBoxesFilteredPub_, 1, 0, 0.5);
        
        // STEP 3: Fuse LiDAR and visual bounding boxes
        // init processed flags

        this->BboxesMerger(
            visualBBoxesTemp,                     // group1 = visual
            lidarBBoxesFiltered,                  // group2 = lidar

            visualPcClustersTemp,                 // cluster visual
            visualPcClusterCentersTemp,
            visualPcClusterStdsTemp,

            lidarPcClustersFiltered,              // cluster lidar
            lidarPcClusterCentersFiltered,
            lidarPcClusterStdsFiltered,

            filteredBBoxesTemp,
            filteredPcClustersTemp,
            filteredPcClusterCentersTemp,
            filteredPcClusterStdsTemp,

            this->lidarVisualmergingFlag_,                            
            this->visualUnmergedFlag_,
            this->lidarUnmergedFlag_,                              

            this->lidarVisualboxIOUThresh_,
            this->lidarVisualboxIOVThresh_,
            this->lidarVisualMergerLeafOnly_
        );

        // =========================
        // STEP 4 (optional): FINAL MERGE — extra mergeNestedGroup pass on the fully
        // fused filtered bboxes, before YOLO association. Collapses residual duplicates
        // that survived the UV/DBSCAN and visual/LiDAR merging stages.
        // Uses the same samegroupIOU / samegroupIOV thresholds.
        // =========================
        if (this->finalMergeFlag_){
            std::vector<joDetector::box3D> finalMergedBBoxes;
            std::vector<std::vector<Eigen::Vector3d>> finalMergedClusters;
            std::vector<Eigen::Vector3d> finalMergedCenters;
            std::vector<Eigen::Vector3d> finalMergedStds;

            this->mergeNestedGroup(
                filteredBBoxesTemp,
                filteredPcClustersTemp,
                filteredPcClusterCentersTemp,
                filteredPcClusterStdsTemp,

                finalMergedBBoxes,
                finalMergedClusters,
                finalMergedCenters,
                finalMergedStds
            );

            filteredBBoxesTemp         = finalMergedBBoxes;
            filteredPcClustersTemp     = finalMergedClusters;
            filteredPcClusterCentersTemp = finalMergedCenters;
            filteredPcClusterStdsTemp  = finalMergedStds;
        }

        removeOversizedBoxes(filteredBBoxesTemp,
                               filteredPcClustersTemp,
                               filteredPcClusterCentersTemp,
                               filteredPcClusterStdsTemp);

        this->filteredBBoxesBeforeYolo_ = filteredBBoxesTemp; // ready for visualization / next steps

        // =========================
        // YOLO DYNAMIC CLASSIFICATION (depth-projected point cloud)
        // =========================
        // Pipeline:
        //   1) For each YOLO detection of a target class, build a 3D point cloud
        //      by unprojecting depth pixels inside the YOLO rect.
        //   2) Filter background: keep only foreground points (within tolerance
        //      of the 10th-percentile depth).
        //   3) For each filtered point, check which filteredBBoxesTemp it falls in.
        //   4) If a bbox captures enough of the filtered YOLO points → mark dynamic.
        std::vector<Eigen::Vector3d> allYoloPoints; // collect for visualization
        if (!this->yoloDetectionResults_.detections.empty() && !this->depthImage_.empty())
        {
            // Precompute constants for depth → 3D → world
            const double inv_factor = 1.0 / this->depthScale_;
            const double inv_fx = 1.0 / this->fx_;
            const double inv_fy = 1.0 / this->fy_;
            const int dCols = this->depthImage_.cols;
            const int dRows = this->depthImage_.rows;

            // Mapping from color pixel to depth pixel (co-located cameras):
            //   u_depth = fx_/fxC_ * (u_color - cxC_) + cx_
            //   v_depth = fy_/fyC_ * (v_color - cyC_) + cy_
            const double scaleU = this->fx_ / this->fxC_;
            const double scaleV = this->fy_ / this->fyC_;
            const double offU  = this->cx_ - scaleU * this->cxC_;
            const double offV  = this->cy_ - scaleV * this->cyC_;

            for (size_t i = 0; i < this->yoloDetectionResults_.detections.size(); ++i){
                const auto& yoloDet = this->yoloDetectionResults_.detections[i];

                // Only process target dynamic classes
                std::string detClass;
                if (!yoloDet.results.empty()){
                    detClass = yoloDet.results[0].hypothesis.class_id;
                }
                bool isDynamicClass = false;
                for (const auto& cls : this->yoloDynamicClasses_){
                    if (cls == detClass){ isDynamicClass = true; break; }
                }
                if (!isDynamicClass) continue;

                // YOLO rect in color image coordinates
                int tlX = 0, tlY = 0;
                if (!yoloDet.results.empty()){
                    tlX = static_cast<int>(yoloDet.results[0].pose.pose.position.x);
                    tlY = static_cast<int>(yoloDet.results[0].pose.pose.position.y);
                }
                const int brX = tlX + static_cast<int>(yoloDet.bbox.size_x);
                const int brY = tlY + static_cast<int>(yoloDet.bbox.size_y);
                if (brX <= tlX || brY <= tlY) continue;

                // Visualize YOLO detection
                if (!this->detectedColorImage_.empty()){
                    cv::Rect bboxVis(tlX, tlY, brX - tlX, brY - tlY);
                    cv::rectangle(this->detectedColorImage_, bboxVis, cv::Scalar(255, 0, 0), 5, 8, 0);
                    cv::putText(this->detectedColorImage_, detClass,
                                cv::Point(bboxVis.x, bboxVis.y - 10),
                                cv::FONT_HERSHEY_SIMPLEX, 1.0,
                                cv::Scalar(255, 0, 0), 2, 8);
                }

                // --------------------------------------------------
                // STEP 1: Build YOLO 3D point cloud from depth image
                // --------------------------------------------------
                // Map YOLO rect corners to depth image pixel ROI
                int dTlU = static_cast<int>(std::floor(scaleU * tlX + offU));
                int dTlV = static_cast<int>(std::floor(scaleV * tlY + offV));
                int dBrU = static_cast<int>(std::ceil (scaleU * brX + offU));
                int dBrV = static_cast<int>(std::ceil (scaleV * brY + offV));
                dTlU = std::max(dTlU, 0);  dTlV = std::max(dTlV, 0);
                dBrU = std::min(dBrU, dCols); dBrV = std::min(dBrV, dRows);

                if (dBrU <= dTlU || dBrV <= dTlV) continue;

                // Ellipse mask: inscribed in the depth ROI; corners are excluded
                // because they almost always contain background from adjacent objects.
                const double ellCu = (dTlU + dBrU) * 0.5;
                const double ellCv = (dTlV + dBrV) * 0.5;
                const double ellRu = (dBrU - dTlU) * 0.5; // always >= 0.5 (ROI is non-empty)
                const double ellRv = (dBrV - dTlV) * 0.5;

                // Collect 3D points inside the inscribed ellipse of the depth ROI.
                std::vector<Eigen::Vector3d> yoloPoints;
                std::vector<double> yoloDepths; // kept for indoor percentile filter
                yoloPoints.reserve((dBrU - dTlU) * (dBrV - dTlV));
                yoloDepths.reserve(yoloPoints.capacity());

                for (int vv = dTlV; vv < dBrV; ++vv){
                    const uint16_t* rowPtr = this->depthImage_.ptr<uint16_t>(vv);
                    for (int uu = dTlU; uu < dBrU; ++uu){
                        // Skip pixels outside the inscribed ellipse (corners = background)
                        const double du = (uu - ellCu) / ellRu;
                        const double dv = (vv - ellCv) / ellRv;
                        if (du*du + dv*dv > 1.0) continue;

                        if (rowPtr[uu] == 0) continue;
                        const double d = rowPtr[uu] * inv_factor;
                        if (d < this->depthMinValue_ || d > this->depthMaxValue_) continue;

                        // Depth pixel → 3D camera frame
                        Eigen::Vector3d ptCam;
                        ptCam(0) = (uu - this->cx_) * d * inv_fx;
                        ptCam(1) = (vv - this->cy_) * d * inv_fy;
                        ptCam(2) = d;

                        // Camera frame → world frame
                        Eigen::Vector3d ptWorld = this->orientationDepth_ * ptCam + this->positionDepth_;

                        yoloPoints.push_back(ptWorld);
                        yoloDepths.push_back(d);
                    }
                }

                // 2D fallback: project each 3D bbox's 8 corners onto the color image
                // and compare the projected rect with the YOLO rect. This is used both
                // when the whole YOLO ROI has no useful depth and per-box when only a few
                // YOLO depth points fall inside a candidate 3D box.
                // Multiple 3D bboxes can match the same YOLO detection.
                auto computeProjectedYoloIou = [&](const joDetector::box3D& bb3d) -> double {
                    const double hx3 = bb3d.x_width * 0.5;
                    const double hy3 = bb3d.y_width * 0.5;
                    const double hz3 = bb3d.z_width * 0.5;

                    const double cxArr[2] = {bb3d.x - hx3, bb3d.x + hx3};
                    const double cyArr[2] = {bb3d.y - hy3, bb3d.y + hy3};
                    const double czArr[2] = {bb3d.z - hz3, bb3d.z + hz3};

                    double projMinU = std::numeric_limits<double>::max();
                    double projMinV = std::numeric_limits<double>::max();
                    double projMaxU = std::numeric_limits<double>::lowest();
                    double projMaxV = std::numeric_limits<double>::lowest();
                    bool anyValid = false;

                    for (int ia = 0; ia < 2; ++ia)
                    for (int ib = 0; ib < 2; ++ib)
                    for (int ic = 0; ic < 2; ++ic){
                        Eigen::Vector3d corner(cxArr[ia], cyArr[ib], czArr[ic]);
                        Eigen::Vector3d camPt = this->orientationColor_.transpose() * (corner - this->positionColor_);
                        if (camPt(2) <= 0.0) continue;
                        const double u = this->fxC_ * camPt(0) / camPt(2) + this->cxC_;
                        const double v = this->fyC_ * camPt(1) / camPt(2) + this->cyC_;
                        if (u < projMinU) projMinU = u;
                        if (u > projMaxU) projMaxU = u;
                        if (v < projMinV) projMinV = v;
                        if (v > projMaxV) projMaxV = v;
                        anyValid = true;
                    }

                    if (!anyValid || projMaxU <= projMinU || projMaxV <= projMinV) return 0.0;

                    const double interMinU = std::max(projMinU, static_cast<double>(tlX));
                    const double interMaxU = std::min(projMaxU, static_cast<double>(brX));
                    const double interMinV = std::max(projMinV, static_cast<double>(tlY));
                    const double interMaxV = std::min(projMaxV, static_cast<double>(brY));

                    if (interMaxU <= interMinU || interMaxV <= interMinV) return 0.0;

                    const double interArea = (interMaxU - interMinU) * (interMaxV - interMinV);
                    const double projArea  = (projMaxU - projMinU) * (projMaxV - projMinV);
                    const double yoloArea  = static_cast<double>((brX - tlX) * (brY - tlY));
                    const double unionArea = projArea + yoloArea - interArea;
                    if (unionArea <= 0.0) return 0.0;

                    return interArea / unionArea;
                };

                auto tryIouFallback = [&](){
                    for (size_t j2 = 0; j2 < filteredBBoxesTemp.size(); ++j2){
                        if (computeProjectedYoloIou(filteredBBoxesTemp[j2]) >= this->yolo2dIouThreshold_)
                            filteredBBoxesTemp[j2].is_yolo_candidate = true;
                    }
                };

                if (yoloPoints.empty()){
                    tryIouFallback();
                    continue;
                }

                // --------------------------------------------------
                // STEP 2: Background filtering
                // --------------------------------------------------
                // Indoor: 10th-percentile depth + tight tolerance to strip walls/ceiling
                //   directly behind the object.
                // Outdoor: 20th-percentile depth + wider tolerance to remove distant
                //   background (terrain, buildings) while preserving the full depth
                //   range of large objects (vehicles, cyclists seen from an angle).
                std::vector<Eigen::Vector3d> filteredYoloPoints;
                filteredYoloPoints.reserve(yoloPoints.size());

                {
                    std::vector<double> sortedDepths = yoloDepths;
                    std::sort(sortedDepths.begin(), sortedDepths.end());
                    const size_t n = sortedDepths.size();
                    const double foregroundDepth = this->isIndoor_
                        ? sortedDepths[n / 20]   // 5th percentile — tight, walls right behind
                        : sortedDepths[n / 5];   // 20th percentile — softer, objects span more depth
                    const double tolerance = this->isIndoor_
                        ? this->yoloDepthTolerance_
                        : this->yoloOutdoorDepthTolerance_;
                    const double maxAllowedDepth = foregroundDepth + tolerance;
                    for (size_t k = 0; k < yoloPoints.size(); ++k){
                        if (yoloDepths[k] <= maxAllowedDepth &&
                            yoloPoints[k].z() >= this->groundHeight_ &&
                            yoloPoints[k].z() <= this->roofHeight_)
                            filteredYoloPoints.push_back(yoloPoints[k]);
                    }
                }

                if (filteredYoloPoints.empty()){
                    tryIouFallback();
                    continue;
                }

                // Accumulate for visualization
                allYoloPoints.insert(allYoloPoints.end(), filteredYoloPoints.begin(), filteredYoloPoints.end());

                // --------------------------------------------------
                // STEP 3 & 4: Check which bboxes contain these points
                // --------------------------------------------------
                for (size_t j = 0; j < filteredBBoxesTemp.size(); ++j){
                    const auto& bb = filteredBBoxesTemp[j];
                    const double hx = bb.x_width * 0.5;
                    const double hy = bb.y_width * 0.5;
                    const double hz = bb.z_width * 0.5;

                    std::vector<Eigen::Vector3d> yoloInsidePoints;
                    for (const auto& pt : filteredYoloPoints){
                        if (std::abs(pt(0) - bb.x) <= hx &&
                            std::abs(pt(1) - bb.y) <= hy &&
                            std::abs(pt(2) - bb.z) <= hz){
                            yoloInsidePoints.push_back(pt);
                        }
                    }

                    const int totalFiltered = static_cast<int>(filteredYoloPoints.size());
                    const int yoloInsideCount = static_cast<int>(yoloInsidePoints.size());
                    const double fraction = static_cast<double>(yoloInsideCount) / static_cast<double>(totalFiltered);
                    const double boxVolume = std::max(bb.x_width * bb.y_width * bb.z_width, 1e-6);
                    const double yoloPointDensity = static_cast<double>(yoloInsideCount) / boxVolume;
                    const bool sparseLargeBox =
                        boxVolume >= this->yoloSparseLargeBoxVolumeThresh_ &&
                        (yoloInsideCount < this->yoloSparseLargeMinPoints_ ||
                         yoloPointDensity < this->yoloSparseLargeMinDensity_);

                    // Height ratio filter: if the 3D box is much taller than the vertical
                    // span of the YOLO points inside it, the box is almost certainly a large
                    // static structure (wall, building) that the YOLO detection only partially
                    // hits. Clamp the YOLO Z span to yoloMinHeightSpan_ to avoid near-zero.
                    bool boxTooTall = false;
                    if (!yoloInsidePoints.empty()){
                        double yoloZMin = std::numeric_limits<double>::max();
                        double yoloZMax = std::numeric_limits<double>::lowest();
                        for (const auto& pt : yoloInsidePoints){
                            if (pt(2) < yoloZMin) yoloZMin = pt(2);
                            if (pt(2) > yoloZMax) yoloZMax = pt(2);
                        }
                        const double yoloZSpan = std::max(yoloZMax - yoloZMin, this->yoloMinHeightSpan_);
                        boxTooTall = bb.z_width > yoloZSpan * this->yoloMaxHeightRatio_;
                    }

                    const bool pointBasedCandidate =
                        !yoloInsidePoints.empty() &&
                        fraction >= this->yoloPointFractionThresh_ &&
                        !sparseLargeBox &&
                        !boxTooTall;

                    const bool sparseDepthSupport =
                        yoloInsideCount < this->yoloSparseLargeMinPoints_ ||
                        fraction < this->yoloPointFractionThresh_;
                    const bool iouFallbackCandidate =
                        !pointBasedCandidate &&
                        sparseDepthSupport &&
                        computeProjectedYoloIou(bb) >= this->yolo2dIouThreshold_;

                    if (pointBasedCandidate || iouFallbackCandidate){
                        filteredBBoxesTemp[j].is_yolo_candidate = true;

                        if (!pointBasedCandidate) continue;

                        // --- X/Y resize (optional) ---
                        double newCenterX = filteredBBoxesTemp[j].x;
                        double newCenterY = filteredBBoxesTemp[j].y;
                        double newXWidth   = filteredBBoxesTemp[j].x_width;
                        double newYWidth   = filteredBBoxesTemp[j].y_width;

                        if (this->yoloXYResize_){
                            double minX = std::numeric_limits<double>::max(), maxX = std::numeric_limits<double>::lowest();
                            double minY = std::numeric_limits<double>::max(), maxY = std::numeric_limits<double>::lowest();
                            for (const auto& pt : yoloInsidePoints){
                                if (pt(0) < minX) minX = pt(0);
                                if (pt(0) > maxX) maxX = pt(0);
                                if (pt(1) < minY) minY = pt(1);
                                if (pt(1) > maxY) maxY = pt(1);
                            }
                            newXWidth   = maxX - minX;
                            newYWidth   = maxY - minY;
                            newCenterX  = (minX + maxX) * 0.5;
                            newCenterY  = (minY + maxY) * 0.5;
                            filteredBBoxesTemp[j].x       = newCenterX;
                            filteredBBoxesTemp[j].y       = newCenterY;
                            filteredBBoxesTemp[j].x_width = newXWidth;
                            filteredBBoxesTemp[j].y_width = newYWidth;
                        }

                        // --- Height correction (always active) ---
                        double minZ = std::numeric_limits<double>::max();
                        double maxZ = std::numeric_limits<double>::lowest();
                        for (const auto& pt : yoloInsidePoints){
                            if (pt(2) < minZ) minZ = pt(2);
                            if (pt(2) > maxZ) maxZ = pt(2);
                        }
                        const double yoloZWidth = maxZ - minZ;
                        const double yoloZCenter = (minZ + maxZ) * 0.5;
                        const double heightDiff = std::abs(filteredBBoxesTemp[j].z_width - yoloZWidth);

                        if (yoloZWidth > 0.0 && heightDiff >= this->yoloHeightCorrectionThreshold_){
                            filteredBBoxesTemp[j].z_width = yoloZWidth;
                            filteredBBoxesTemp[j].z = yoloZCenter;
                        }

                        // --- Cluster re-filter (only when x/y was resized) ---
                        if (this->yoloXYResize_){
                            const double nhx = newXWidth * 0.5;
                            const double nhy = newYWidth * 0.5;
                            const double nhz = filteredBBoxesTemp[j].z_width * 0.5;
                            const double ncz = filteredBBoxesTemp[j].z;

                            std::vector<Eigen::Vector3d> newCluster;
                            for (const auto& pt : filteredPcClustersTemp[j]){
                                if (std::abs(pt(0) - newCenterX) <= nhx &&
                                    std::abs(pt(1) - newCenterY) <= nhy &&
                                    std::abs(pt(2) - ncz) <= nhz)
                                    newCluster.push_back(pt);
                            }
                            filteredPcClustersTemp[j] = newCluster;

                            if (!newCluster.empty()){
                                Eigen::Vector3d center = Eigen::Vector3d::Zero();
                                for (const auto& pt : newCluster) center += pt;
                                center /= static_cast<double>(newCluster.size());
                                filteredPcClusterCentersTemp[j] = center;

                                Eigen::Vector3d stddev = Eigen::Vector3d::Zero();
                                for (const auto& pt : newCluster)
                                    stddev += (pt - center).cwiseAbs2();
                                filteredPcClusterStdsTemp[j] = (stddev / static_cast<double>(newCluster.size())).cwiseSqrt();
                            }
                        }
                    }
                }
            }
        }
        // Publish YOLO foreground points for visualization
        this->publishPoints(allYoloPoints, this->yoloPointsPub_);

        // Consume YOLO detections: clear so the same detection is not reapplied
        // to future detection-loop iterations that run before the next YOLO frame.
        // The is_yolo_candidate flag is now sticky in boxHist_ (via kalmanFilterAndUpdateHist),
        // so the track survives without needing the raw YOLO result again.
        this->yoloDetectionResults_.detections.clear();

        // Re-apply indoor size filter: YOLO may have expanded box dimensions after the
        // first removeOversizedBoxes pass (x/y resize and height correction).
        removeOversizedBoxes(filteredBBoxesTemp,
                               filteredPcClustersTemp,
                               filteredPcClusterCentersTemp,
                               filteredPcClusterStdsTemp);

        this->filteredBBoxes_ = filteredBBoxesTemp;
        this->filteredPcClusters_ = filteredPcClustersTemp;
        this->filteredPcClusterCenters_ = filteredPcClusterCentersTemp;
        this->filteredPcClusterStds_ = filteredPcClusterStdsTemp;
    }


    void dynamicDetector::transformUVBBoxes(std::vector<joDetector::box3D>& bboxes){
        bboxes.clear();
        for(size_t i = 0; i < this->uvDetector_->box3Ds.size(); ++i){
            joDetector::box3D bbox;
            double x = this->uvDetector_->box3Ds[i].x; 
            double y = this->uvDetector_->box3Ds[i].y;
            double z = this->uvDetector_->box3Ds[i].z;
            double xWidth = this->uvDetector_->box3Ds[i].x_width;
            double yWidth = this->uvDetector_->box3Ds[i].y_width;
            double zWidth = this->uvDetector_->box3Ds[i].z_width;

            Eigen::Vector3d center (x, y, z);
            Eigen::Vector3d size (xWidth, yWidth, zWidth);
            Eigen::Vector3d newCenter, newSize;

            this->transformBBox(center, size, this->positionDepth_, this->orientationDepth_, newCenter, newSize);

            // assign values to bounding boxes in the map frame
            bbox.x = newCenter(0);
            bbox.y = newCenter(1);
            bbox.z = newCenter(2);
            bbox.x_width = newSize(0);
            bbox.y_width = newSize(1);
            bbox.z_width = newSize(2);
            bboxes.push_back(bbox);            
        }        
    }

    void dynamicDetector::projectDepthImage(){
                // Debug: Print image size and skipPixel_
                // RCLCPP_INFO(this->nh_->get_logger(), "[DEBUG] projectDepthImage: imgCols_=%d, imgRows_=%d, skipPixel_=%d", this->imgCols_, this->imgRows_, this->skipPixel_);
                if (this->imgCols_ <= 0 || this->imgRows_ <= 0 || this->skipPixel_ <= 0) {
                    RCLCPP_ERROR(this->nh_->get_logger(), "[ERROR] Invalid image size or skipPixel_: imgCols_=%d, imgRows_=%d, skipPixel_=%d", this->imgCols_, this->imgRows_, this->skipPixel_);
                }
                if (this->projPoints_.size() == 0 || this->pointsDepth_.size() == 0) {
                    RCLCPP_ERROR(this->nh_->get_logger(), "[ERROR] projPoints_ or pointsDepth_ not allocated! projPoints_.size()=%zu, pointsDepth_.size()=%zu", this->projPoints_.size(), this->pointsDepth_.size());
                }
        this->projPointsNum_ = 0;

        int cols = this->depthImage_.cols;
        int rows = this->depthImage_.rows;
        uint16_t* rowPtr;

        Eigen::Vector3d currPointCam, currPointMap;
        double depth;
        const double inv_factor = 1.0 / this->depthScale_;
        const double inv_fx = 1.0 / this->fx_;
        const double inv_fy = 1.0 / this->fy_;

        // iterate through each pixel in the depth image
        for (int v=this->depthFilterMargin_; v<rows-this->depthFilterMargin_; v=v+this->skipPixel_){ // row
            rowPtr = this->depthImage_.ptr<uint16_t>(v) + this->depthFilterMargin_;
            for (int u=this->depthFilterMargin_; u<cols-this->depthFilterMargin_; u=u+this->skipPixel_){ // column
                depth = (*rowPtr) * inv_factor;
                
                if (*rowPtr == 0) {
                    depth = this->raycastMaxLength_ + 0.1;
                } else if (depth < this->depthMinValue_) {
                    continue;
                } else if (depth > this->depthMaxValue_) {
                    depth = this->raycastMaxLength_ + 0.1;
                }
                rowPtr =  rowPtr + this->skipPixel_;

                // get 3D point in camera frame
                currPointCam(0) = (u - this->cx_) * depth * inv_fx;
                currPointCam(1) = (v - this->cy_) * depth * inv_fy;
                currPointCam(2) = depth;
                currPointMap = this->orientationDepth_ * currPointCam + this->positionDepth_; // transform to map coordinate

                this->projPoints_[this->projPointsNum_] = currPointMap;
                this->pointsDepth_[this->projPointsNum_] = depth;
                this->projPointsNum_ = this->projPointsNum_ + 1;
            }
        } 
    }

    void dynamicDetector::filterPoints(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& filteredPoints){
        // currently there is only one filtered (might include more in the future)
        std::vector<Eigen::Vector3d> voxelFilteredPoints;
        this->voxelFilter(points, voxelFilteredPoints);

        filteredPoints.clear();
        // std::cout<<"Ground Height is: "<<this->groundHeight_<<", Roof Height is: "<<this->roofHeight_<<std::endl;
        for (const auto& point : voxelFilteredPoints){
            if (point.z() <= this->roofHeight_ && point.z() >= this->groundHeight_ && !this->isInsideAnyWall(point)){
                filteredPoints.push_back(point);
            }
        }
    }


    void dynamicDetector::clusterPointsAndBBoxes(
        const std::vector<Eigen::Vector3d>& points,
        std::vector<joDetector::box3D>& bboxes,
        std::vector<std::vector<Eigen::Vector3d>>& pcClusters,
        std::vector<Eigen::Vector3d>& pcClusterCenters,
        std::vector<Eigen::Vector3d>& pcClusterStds)
    {
        bboxes.clear();
        pcClusters.clear();
        pcClusterCenters.clear();
        pcClusterStds.clear();


        std::vector<joDetector::Point> pointsDB;
        this->eigenToDBPointVec(points, pointsDB, points.size());
        this->dbCluster_.reset(new DBSCAN(
            this->dbMinPointsCluster_,
            this->dbEpsilon_,
            pointsDB));

        this->dbCluster_->setRefinementParams(
            this->visualDbscanRefinementEnable_,
            this->dbscanRefineMaxDiagonal_,
            this->dbscanRefineMinDensity_,
            this->dbscanRefineSplitMinPts_,
            this->dbscanRefineSplitEps_,
            this->dbscanRefineMinSubclusterPts_,
            this->dbscanRefineAxisSliceWidth_,
            this->dbscanRefineMaxDepth_,
            this->dbscanRefineRecursive_,
            this->dbscanRefineMinBoxVolume_);

        this->dbCluster_->run();

        if (this->visualDbscanRefinementEnable_) {
            // Use refined clusters with AABB
            std::vector<joDetector::ClusterRefined> refinedClusters;
            this->dbCluster_->getRefinedClustersWithAABB(refinedClusters);
            for (size_t i = 0; i < refinedClusters.size(); ++i) {
                const auto& ref = refinedClusters[i];
                joDetector::box3D box;
                box.id = static_cast<int>(i);
                box.x = (ref.box.min_pt(0) + ref.box.max_pt(0)) / 2.0;
                box.y = (ref.box.min_pt(1) + ref.box.max_pt(1)) / 2.0;
                box.z = (ref.box.min_pt(2) + ref.box.max_pt(2)) / 2.0;
                box.x_width = std::max(ref.box.max_pt(0) - ref.box.min_pt(0), 0.1);
                box.y_width = std::max(ref.box.max_pt(1) - ref.box.min_pt(1), 0.1);
                box.z_width = ref.box.max_pt(2) - ref.box.min_pt(2);
                box.Vx = 0.0;
                box.Vy = 0.0;

                if (box.x_width > this->maxObjectSize_(0) ||
                    box.y_width > this->maxObjectSize_(1) ||
                    box.z_width > this->maxObjectSize_(2)) {
                    continue;
                }

                bboxes.push_back(box);
                pcClusters.push_back(ref.points);
            }
        } else {
            // Use raw clusters as before
            int clusterNum = 0;
            for (size_t i = 0; i < this->dbCluster_->m_points.size(); ++i) {
                joDetector::Point pDB = this->dbCluster_->m_points[i];
                if (pDB.clusterID > clusterNum) {
                    clusterNum = pDB.clusterID;
                }
            }

            std::vector<std::vector<Eigen::Vector3d>> pcClustersTemp;
            pcClustersTemp.resize(clusterNum);

            for (size_t i = 0; i < this->dbCluster_->m_points.size(); ++i) {
                joDetector::Point pDB = this->dbCluster_->m_points[i];
                if (pDB.clusterID > 0) {
                    Eigen::Vector3d p = this->dbPointToEigen(pDB);
                    pcClustersTemp[pDB.clusterID - 1].push_back(p);
                }
            }

            for (size_t i = 0; i < pcClustersTemp.size(); ++i) {
                joDetector::box3D box;
                double xmin = pcClustersTemp[i][0](0);
                double ymin = pcClustersTemp[i][0](1);
                double zmin = pcClustersTemp[i][0](2);
                double xmax = pcClustersTemp[i][0](0);
                double ymax = pcClustersTemp[i][0](1);
                double zmax = pcClustersTemp[i][0](2);

                for (size_t j = 0; j < pcClustersTemp[i].size(); ++j) {
                    xmin = (pcClustersTemp[i][j](0) < xmin) ? pcClustersTemp[i][j](0) : xmin;
                    ymin = (pcClustersTemp[i][j](1) < ymin) ? pcClustersTemp[i][j](1) : ymin;
                    zmin = (pcClustersTemp[i][j](2) < zmin) ? pcClustersTemp[i][j](2) : zmin;
                    xmax = (pcClustersTemp[i][j](0) > xmax) ? pcClustersTemp[i][j](0) : xmax;
                    ymax = (pcClustersTemp[i][j](1) > ymax) ? pcClustersTemp[i][j](1) : ymax;
                    zmax = (pcClustersTemp[i][j](2) > zmax) ? pcClustersTemp[i][j](2) : zmax;
                }

                box.id = static_cast<int>(i);
                box.x = (xmax + xmin) / 2.0;
                box.y = (ymax + ymin) / 2.0;
                box.z = (zmax + zmin) / 2.0;
                box.x_width = (xmax - xmin) > 0.1 ? (xmax - xmin) : 0.1;
                box.y_width = (ymax - ymin) > 0.1 ? (ymax - ymin) : 0.1;
                box.z_width = (zmax - zmin);
                box.Vx = 0.0;
                box.Vy = 0.0;

                if (box.x_width > this->maxObjectSize_(0) ||
                    box.y_width > this->maxObjectSize_(1) ||
                    box.z_width > this->maxObjectSize_(2)) {
                    continue;
                }

                bboxes.push_back(box);
                pcClusters.push_back(pcClustersTemp[i]);
            }
        }

        // Calculate cluster centers and stds for all clusters
        for (size_t i = 0; i < pcClusters.size(); ++i) {
            Eigen::Vector3d pcClusterCenter(0., 0., 0.);
            Eigen::Vector3d pcClusterStd(0., 0., 0.);
            this->calcPcFeat(pcClusters[i], pcClusterCenter, pcClusterStd);
            pcClusterCenters.push_back(pcClusterCenter);
            pcClusterStds.push_back(pcClusterStd);
        }

        if (bboxes.size() != pcClusters.size() ||
            bboxes.size() != pcClusterCenters.size() ||
            bboxes.size() != pcClusterStds.size())
        {
            RCLCPP_ERROR(this->nh_->get_logger(),
                "[dynamicDetector] Size mismatch after clustering: boxes=%zu clusters=%zu centers=%zu stds=%zu",
                bboxes.size(), pcClusters.size(), pcClusterCenters.size(), pcClusterStds.size());
        }
    }

    void dynamicDetector::voxelFilter(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& filteredPoints){
        const double res = this->voxelSize_; // resolution of voxel (now tunable)
        int xVoxels = ceil(2*this->localSensorRange_(0)/res); int yVoxels = ceil(2*this->localSensorRange_(1)/res); int zVoxels = ceil(2*this->localSensorRange_(2)/res);
        int totalVoxels = xVoxels * yVoxels * zVoxels;
        // std::vector<bool> voxelOccupancyVec (totalVoxels, false);
        std::vector<int> voxelOccupancyVec (totalVoxels, 0);

        // Iterate through each points in the cloud
        filteredPoints.clear();
        
        for (int i=0; i<this->projPointsNum_; ++i){
            Eigen::Vector3d p = points[i];

            if (this->isInFilterRange(p) and p(2) >= this->groundHeight_ and this->pointsDepth_[i] <= this->raycastMaxLength_ and !this->isInsideAnyWall(p)){
                // find the corresponding voxel id in the vector and check whether it is occupied
                int pID = this->posToAddress(p, res);

                // add one point
                voxelOccupancyVec[pID] +=1;

                // add only if thresh points are found
                if (voxelOccupancyVec[pID] == this->voxelOccThresh_){
                    filteredPoints.push_back(p);
                }
            }
        } 
    }

    void dynamicDetector::calcPcFeat(const std::vector<Eigen::Vector3d>& pcCluster, Eigen::Vector3d& pcClusterCenter, Eigen::Vector3d& pcClusterStd){
        int numPoints = pcCluster.size();
        
        // center
        for (int i=0 ; i<numPoints ; i++){
            pcClusterCenter(0) += pcCluster[i](0)/numPoints;
            pcClusterCenter(1) += pcCluster[i](1)/numPoints;
            pcClusterCenter(2) += pcCluster[i](2)/numPoints;
        }

        // std
        for (int i=0 ; i<numPoints ; i++){
            pcClusterStd(0) += std::pow(pcCluster[i](0) - pcClusterCenter(0),2);
            pcClusterStd(1) += std::pow(pcCluster[i](1) - pcClusterCenter(1),2);
            pcClusterStd(2) += std::pow(pcCluster[i](2) - pcClusterCenter(2),2);
        }        

        // take square root
        pcClusterStd(0) = std::sqrt(pcClusterStd(0)/numPoints);
        pcClusterStd(1) = std::sqrt(pcClusterStd(1)/numPoints);
        pcClusterStd(2) = std::sqrt(pcClusterStd(2)/numPoints);
    }


    double dynamicDetector::calBoxIOU(const joDetector::box3D& box1,
                                    const joDetector::box3D& box2,
                                    bool ignoreZmin)
    {
        // Sanity check input
        if (!std::isfinite(box1.x) || !std::isfinite(box1.y) || !std::isfinite(box1.z) ||
            !std::isfinite(box1.x_width) || !std::isfinite(box1.y_width) || !std::isfinite(box1.z_width) ||
            !std::isfinite(box2.x) || !std::isfinite(box2.y) || !std::isfinite(box2.z) ||
            !std::isfinite(box2.x_width) || !std::isfinite(box2.y_width) || !std::isfinite(box2.z_width))
        {
            std::cout << "[WARN] calBoxIOU: NaN/Inf detected in input boxes" << std::endl;
            return 0.0;
        }

        if (box1.x_width <= 0.0 || box1.y_width <= 0.0 || box1.z_width <= 0.0 ||
            box2.x_width <= 0.0 || box2.y_width <= 0.0 || box2.z_width <= 0.0)
        {
            return 0.0;
        }

        // Box 1 bounds
        const double b1_xmin = box1.x - box1.x_width * 0.5;
        const double b1_xmax = box1.x + box1.x_width * 0.5;
        const double b1_ymin = box1.y - box1.y_width * 0.5;
        const double b1_ymax = box1.y + box1.y_width * 0.5;

        double b1_zmin = box1.z - box1.z_width * 0.5;
        double b1_zmax = box1.z + box1.z_width * 0.5;

        // Box 2 bounds
        const double b2_xmin = box2.x - box2.x_width * 0.5;
        const double b2_xmax = box2.x + box2.x_width * 0.5;
        const double b2_ymin = box2.y - box2.y_width * 0.5;
        const double b2_ymax = box2.y + box2.y_width * 0.5;

        double b2_zmin = box2.z - box2.z_width * 0.5;
        double b2_zmax = box2.z + box2.z_width * 0.5;

        // If requested, do not consider boxes starting from z=0:
        // clamp lower face to groundHeight_
        if (ignoreZmin)
        {
            b1_zmin = std::max(b1_zmin, this->groundHeight_);
            b2_zmin = std::max(b2_zmin, this->groundHeight_);
        }

        // Recompute valid heights after clamp
        const double b1_zw = b1_zmax - b1_zmin;
        const double b2_zw = b2_zmax - b2_zmin;

        if (b1_zw <= 0.0 || b2_zw <= 0.0)
        {
            return 0.0;
        }

        const double box1Volume = box1.x_width * box1.y_width * b1_zw;
        const double box2Volume = box2.x_width * box2.y_width * b2_zw;

        if (!std::isfinite(box1Volume) || !std::isfinite(box2Volume) ||
            box1Volume <= 0.0 || box2Volume <= 0.0)
        {
            return 0.0;
        }

        // Intersection
        const double inter_x = std::max(0.0, std::min(b1_xmax, b2_xmax) - std::max(b1_xmin, b2_xmin));
        const double inter_y = std::max(0.0, std::min(b1_ymax, b2_ymax) - std::max(b1_ymin, b2_ymin));
        const double inter_z = std::max(0.0, std::min(b1_zmax, b2_zmax) - std::max(b1_zmin, b2_zmin));

        if (inter_x <= 0.0 || inter_y <= 0.0 || inter_z <= 0.0)
        {
            return 0.0;
        }

        const double overlapVolume = inter_x * inter_y * inter_z;
        if (!std::isfinite(overlapVolume) || overlapVolume <= 0.0)
        {
            return 0.0;
        }

        const double unionVolume = box1Volume + box2Volume - overlapVolume;
        if (!std::isfinite(unionVolume) || unionVolume <= 0.0)
        {
            return 0.0;
        }

        const double iou = overlapVolume / unionVolume;
        if (!std::isfinite(iou) || std::isnan(iou))
        {
            return 0.0;
        }

        return std::max(0.0, std::min(1.0, iou));
    }

    double dynamicDetector::calBoxIOV(const joDetector::box3D& box1,
                                    const joDetector::box3D& box2,
                                    bool ignoreZmin)
    {
        // Sanity check input
        if (!std::isfinite(box1.x) || !std::isfinite(box1.y) || !std::isfinite(box1.z) ||
            !std::isfinite(box1.x_width) || !std::isfinite(box1.y_width) || !std::isfinite(box1.z_width) ||
            !std::isfinite(box2.x) || !std::isfinite(box2.y) || !std::isfinite(box2.z) ||
            !std::isfinite(box2.x_width) || !std::isfinite(box2.y_width) || !std::isfinite(box2.z_width))
        {
            std::cout << "[WARN] calBoxIOV: NaN/Inf detected in input boxes" << std::endl;
            return 0.0;
        }

        if (box1.x_width <= 0.0 || box1.y_width <= 0.0 || box1.z_width <= 0.0 ||
            box2.x_width <= 0.0 || box2.y_width <= 0.0 || box2.z_width <= 0.0)
        {
            return 0.0;
        }

        // Box 1 bounds
        const double b1_xmin = box1.x - box1.x_width * 0.5;
        const double b1_xmax = box1.x + box1.x_width * 0.5;
        const double b1_ymin = box1.y - box1.y_width * 0.5;
        const double b1_ymax = box1.y + box1.y_width * 0.5;

        double b1_zmin = box1.z - box1.z_width * 0.5;
        double b1_zmax = box1.z + box1.z_width * 0.5;

        // Box 2 bounds
        const double b2_xmin = box2.x - box2.x_width * 0.5;
        const double b2_xmax = box2.x + box2.x_width * 0.5;
        const double b2_ymin = box2.y - box2.y_width * 0.5;
        const double b2_ymax = box2.y + box2.y_width * 0.5;

        double b2_zmin = box2.z - box2.z_width * 0.5;
        double b2_zmax = box2.z + box2.z_width * 0.5;

        // If requested, do not consider boxes starting from z=0:
        // clamp lower face to groundHeight_
        if (ignoreZmin)
        {
            b1_zmin = std::max(b1_zmin, this->groundHeight_);
            b2_zmin = std::max(b2_zmin, this->groundHeight_);
        }

        // Recompute valid heights after clamp
        const double b1_zw = b1_zmax - b1_zmin;
        const double b2_zw = b2_zmax - b2_zmin;

        if (b1_zw <= 0.0 || b2_zw <= 0.0)
        {
            return 0.0;
        }

        const double box1Volume = box1.x_width * box1.y_width * b1_zw;
        if (!std::isfinite(box1Volume) || box1Volume <= 0.0)
        {
            return 0.0;
        }

        // Intersection
        const double inter_x = std::max(0.0, std::min(b1_xmax, b2_xmax) - std::max(b1_xmin, b2_xmin));
        const double inter_y = std::max(0.0, std::min(b1_ymax, b2_ymax) - std::max(b1_ymin, b2_ymin));
        const double inter_z = std::max(0.0, std::min(b1_zmax, b2_zmax) - std::max(b1_zmin, b2_zmin));

        if (inter_x <= 0.0 || inter_y <= 0.0 || inter_z <= 0.0)
        {
            return 0.0;
        }

        const double overlapVolume = inter_x * inter_y * inter_z;
        if (!std::isfinite(overlapVolume) || overlapVolume <= 0.0)
        {
            return 0.0;
        }

        const double iov = overlapVolume / box1Volume;
        if (!std::isfinite(iov) || std::isnan(iov))
        {
            return 0.0;
        }

        return std::max(0.0, std::min(1.0, iov));
    }

    double dynamicDetector::computeBoxIoU2D(const joDetector::box3D& boxA,
                                            const joDetector::box3D& boxB) const
    {
        double axMin = boxA.x - 0.5 * boxA.x_width;
        double axMax = boxA.x + 0.5 * boxA.x_width;
        double ayMin = boxA.y - 0.5 * boxA.y_width;
        double ayMax = boxA.y + 0.5 * boxA.y_width;

        double bxMin = boxB.x - 0.5 * boxB.x_width;
        double bxMax = boxB.x + 0.5 * boxB.x_width;
        double byMin = boxB.y - 0.5 * boxB.y_width;
        double byMax = boxB.y + 0.5 * boxB.y_width;

        double interX = std::max(0.0, std::min(axMax, bxMax) - std::max(axMin, bxMin));
        double interY = std::max(0.0, std::min(ayMax, byMax) - std::max(ayMin, byMin));
        double interArea = interX * interY;

        double areaA = std::max(0.0, boxA.x_width) * std::max(0.0, boxA.y_width);
        double areaB = std::max(0.0, boxB.x_width) * std::max(0.0, boxB.y_width);
        double unionArea = areaA + areaB - interArea;

        if (unionArea <= 1e-9){
            return 0.0;
        }

        return interArea / unionArea;
    } 

    bool dynamicDetector::isNaturalMotion(int trackIdx,
                                        const joDetector::box3D& currDetectedBBox) const
    {
        if (trackIdx < 0 || trackIdx >= static_cast<int>(this->boxHist_.size())){
            return false;
        }

        if (this->boxHist_[trackIdx].empty()){
            return false;
        }

        const joDetector::box3D& prevBBox = this->boxHist_[trackIdx].front();

        const Eigen::Vector3d bboxPos(currDetectedBBox.x, currDetectedBBox.y, currDetectedBBox.z);
        const bool outsideFov = !this->isInCameraFOV(bboxPos);

        const double dx = currDetectedBBox.x - prevBBox.x;
        const double dy = currDetectedBBox.y - prevBBox.y;
        const double obsDist = std::sqrt(dx * dx + dy * dy);

        // Dentro FOV: comportamento originale.
        // Fuori FOV: non rifiutare subito piccoli spostamenti, perché a 20 Hz
        // anche 0.5 m/s produce solo 2.5 cm/frame.
        if (!outsideFov){
            if (!this->trackSteadyObjects_ && obsDist < this->minNaturalMotionDist_){
                return false;
            }
        }

        if (obsDist > this->maxNaturalMotionDist_){
            return false;
        }

        joDetector::kalman_filter predictedFilter = this->filters_[trackIdx];
        predictedFilter.predict(MatrixXd::Zero(6,1));

        const double predX = predictedFilter.output(0);
        const double predY = predictedFilter.output(1);

        const double innovation =
            std::sqrt((currDetectedBBox.x - predX) * (currDetectedBBox.x - predX) +
                    (currDetectedBBox.y - predY) * (currDetectedBBox.y - predY));

        const double maxInnovation = this->getAdaptiveMaxInnovation(trackIdx, currDetectedBBox);

        if (innovation > maxInnovation){
            return false;
        }

        return true;
    }

    double dynamicDetector::computeVelocityDirectionError(int trackIdx,
                                                        const joDetector::box3D& currDetectedBBox) const
    {
        if (trackIdx < 0 || trackIdx >= static_cast<int>(this->boxHist_.size())){
            return std::numeric_limits<double>::infinity();
        }

        if (this->boxHist_[trackIdx].empty()){
            return std::numeric_limits<double>::infinity();
        }

        const joDetector::box3D& prevBBox = this->boxHist_[trackIdx].front();
        const double dt = this->clampPositive(this->dt_, 1e-3);

        const double obsVx = (currDetectedBBox.x - prevBBox.x) / dt;
        const double obsVy = (currDetectedBBox.y - prevBBox.y) / dt;

        joDetector::kalman_filter predictedFilter = this->filters_[trackIdx];
        predictedFilter.predict(MatrixXd::Zero(6,1));

        const double predVx = predictedFilter.output(2);
        const double predVy = predictedFilter.output(3);

        const double obsSpeed = std::sqrt(obsVx * obsVx + obsVy * obsVy);
        const double predSpeed = std::sqrt(predVx * predVx + predVy * predVy);

        // Caso importante:
        // se oggetto osservato e predetto sono praticamente fermi,
        // NON lo consideriamo errore.
        if (obsSpeed < this->stationarySpeedThresh_ &&
            predSpeed < this->stationarySpeedThresh_)
        {
            return 0.0;
        }

        // Se uno è fermo e l’altro no, c’è incoerenza ma non infinita.
        const double velDiff = std::sqrt((obsVx - predVx) * (obsVx - predVx) +
                                        (obsVy - predVy) * (obsVy - predVy));

        // errore direzionale
        double dirErr = 0.0;
        if (obsSpeed >= this->stationarySpeedThresh_ &&
            predSpeed >= this->stationarySpeedThresh_)
        {
            const double dot = obsVx * predVx + obsVy * predVy;
            const double denom = std::max(obsSpeed * predSpeed, 1e-6);
            const double cosSim = std::max(-1.0, std::min(1.0, dot / denom));

            // 0 se perfettamente allineate, 2 se opposte
            dirErr = 1.0 - cosSim;
        }

        return velDiff + dirErr;
    }

    bool dynamicDetector::passesVelocityDirectionGate(int trackIdx,
                                                    const joDetector::box3D& currDetectedBBox,
                                                    bool alreadyConfirmed,
                                                    bool yoloExempt) const
    {
        if (trackIdx < 0 || trackIdx >= static_cast<int>(this->boxHist_.size())){
            return false;
        }

        if (this->boxHist_[trackIdx].empty()){
            return false;
        }

        const joDetector::box3D& prevBBox = this->boxHist_[trackIdx].front();
        const double dt = this->clampPositive(this->dt_, 1e-3);

        const double obsVx = (currDetectedBBox.x - prevBBox.x) / dt;
        const double obsVy = (currDetectedBBox.y - prevBBox.y) / dt;
        const double obsSpeed = std::sqrt(obsVx * obsVx + obsVy * obsVy);

        joDetector::kalman_filter predictedFilter = this->filters_[trackIdx];
        predictedFilter.predict(MatrixXd::Zero(6,1));

        const double predVx = predictedFilter.output(2);
        const double predVy = predictedFilter.output(3);
        const double predSpeed = std::sqrt(predVx * predVx + predVy * predVy);

        const Eigen::Vector3d bboxPos(currDetectedBBox.x,
                                    currDetectedBBox.y,
                                    currDetectedBBox.z);

        const bool outsideFov = !this->isInCameraFOV(bboxPos);

        if ((alreadyConfirmed || this->trackSteadyObjects_) &&
            obsSpeed < this->stationarySpeedThresh_ &&
            predSpeed < this->stationarySpeedThresh_)
        {
            return true;
        }

        const double err = this->computeVelocityDirectionError(trackIdx,
                                                            currDetectedBBox);

        if (yoloExempt){
            if (alreadyConfirmed){
                return err <= this->maxVelocityDirectionErrorTrackedDynamic_;
            }
            return err <= this->maxVelocityDirectionErrorConfirmDynamic_;
        }

        if (alreadyConfirmed){
            // Nuovo comportamento:
            // per track confermate LiDAR-only fuori FOV, non perdiamo subito la track
            // se la persona cambia direzione e la detection resta spazialmente plausibile.
            if (outsideFov){
                joDetector::kalman_filter predictedFilterTurn = this->filters_[trackIdx];
                predictedFilterTurn.predict(MatrixXd::Zero(6,1));

                joDetector::box3D predictedBox = this->boxHist_[trackIdx][0];
                predictedBox.x  = predictedFilterTurn.output(0);
                predictedBox.y  = predictedFilterTurn.output(1);
                predictedBox.Vx = predictedFilterTurn.output(2);
                predictedBox.Vy = predictedFilterTurn.output(3);
                predictedBox.Ax = predictedFilterTurn.output(4);
                predictedBox.Ay = predictedFilterTurn.output(5);

                if (this->isAbruptTurnAllowedOutsideFov(trackIdx,
                                                        predictedBox,
                                                        currDetectedBBox,
                                                        dt))
                {
                    return true;
                }
            }

            return err <= this->maxVelocityDirectionErrorTracked_;
        }

        if (outsideFov){
            return err <= this->maxVelocityDirectionErrorConfirmOutsideFov_;
        }

        return err <= this->maxVelocityDirectionErrorConfirm_;
    }

    bool dynamicDetector::isTrackConfirmedByIdx(int trackIdx) const
    {
        if (trackIdx < 0 || trackIdx >= static_cast<int>(this->confirmedTracks_.size())){
            return false;
        }
        return this->confirmedTracks_[trackIdx];
    }

    double dynamicDetector::getAdaptiveMaxInnovation(int trackIdx,
                                                    const joDetector::box3D& currDetectedBBox) const
    {
        if (currDetectedBBox.is_yolo_candidate){
            return this->maxNaturalInnovationDynamic_;
        }

        if (this->isTrackConfirmedByIdx(trackIdx)){
            return this->maxNaturalInnovationConfirmed_;
        }

        const Eigen::Vector3d bboxPos(currDetectedBBox.x, currDetectedBBox.y, currDetectedBBox.z);
        if (!this->isInCameraFOV(bboxPos)){
            return this->maxNaturalInnovationOutsideFov_;
        }

        return this->maxNaturalInnovation_;
    }

    double dynamicDetector::getAdaptiveMinMatchScore(int trackIdx,
                                                    const joDetector::box3D& currentBox) const
    {
        if (currentBox.is_yolo_candidate){
            return this->minMatchScoreDynamic_;
        }

        if (this->isTrackConfirmedByIdx(trackIdx)){
            return this->minMatchScoreConfirmed_;
        }

        return this->minMatchScore_;
    }

    double dynamicDetector::computeObservedSpeedOverWindow(int trackIdx,
                                                        const joDetector::box3D& currDetectedBBox,
                                                        int window) const
    {
        if (trackIdx < 0 ||
            trackIdx >= static_cast<int>(this->boxHist_.size()) ||
            this->boxHist_[trackIdx].empty())
        {
            return 0.0;
        }

        const int histSize = static_cast<int>(this->boxHist_[trackIdx].size());
        const int k = std::max(1, std::min(window, histSize));

        const joDetector::box3D& pastBBox = this->boxHist_[trackIdx][k - 1];

        const double dx = currDetectedBBox.x - pastBBox.x;
        const double dy = currDetectedBBox.y - pastBBox.y;
        const double dt = this->clampPositive(this->dt_ * static_cast<double>(k), 1e-3);

        return std::sqrt(dx * dx + dy * dy) / dt;
    }

    bool dynamicDetector::isLidarOnlyOutsideFovAssociation(
        const joDetector::box3D& predictedBox,
        const joDetector::box3D& currentBox) const
    {
        const Eigen::Vector3d predPos(predictedBox.x, predictedBox.y, predictedBox.z);
        const Eigen::Vector3d currPos(currentBox.x, currentBox.y, currentBox.z);

        const bool predOutsideFov = !this->isInCameraFOV(predPos);
        const bool currOutsideFov = !this->isInCameraFOV(currPos);

        // Solo caso LiDAR-only fuori camera.
        // Se YOLO è presente o la track ha storia YOLO, lasciamo invariato il comportamento.
        if (!predOutsideFov || !currOutsideFov){
            return false;
        }

        if (predictedBox.is_yolo_candidate || currentBox.is_yolo_candidate){
            return false;
        }

        return true;
    }

    bool dynamicDetector::shouldConfirmTrack(int trackIdx,
                                            const joDetector::box3D& currDetectedBBox,
                                            int newHitStreak) const
    {
        const bool historyHasYolo =
            (trackIdx >= 0 &&
            trackIdx < static_cast<int>(this->boxHist_.size()) &&
            !this->boxHist_[trackIdx].empty() &&
            this->boxHist_[trackIdx][0].is_yolo_candidate);

        // YOLO tracks bypass the motion/stationary bootstrap and are confirmed
        // immediately: they represent known dynamic-class objects.
        if (currDetectedBBox.is_yolo_candidate || historyHasYolo){
            return true;
        }

        if (trackIdx < 0 ||
            trackIdx >= static_cast<int>(this->boxHist_.size()) ||
            this->boxHist_[trackIdx].empty())
        {
            return false;
        }

        const Eigen::Vector3d bboxPos(currDetectedBBox.x, currDetectedBBox.y, currDetectedBBox.z);
        const bool outsideFov = !this->isInCameraFOV(bboxPos);

        const auto& prevBBox = this->boxHist_[trackIdx].front();
        const double dt = this->clampPositive(this->dt_, 1e-3);

        const double dx = currDetectedBBox.x - prevBBox.x;
        const double dy = currDetectedBBox.y - prevBBox.y;
        const double obsSpeed = std::sqrt(dx * dx + dy * dy) / dt;

        // ============================================================
        // NUOVO BOOTSTRAP SOLO FUORI FOV, SOLO NON-YOLO
        // ============================================================
        if (outsideFov && !this->trackSteadyObjects_){

            if (newHitStreak < this->minConfirmHitsOutsideFov_){
                return false;
            }

            const double avgObsSpeed =
                this->computeObservedSpeedOverWindow(trackIdx,
                                                    currDetectedBBox,
                                                    this->outsideFovObsVelWindow_);

            if (avgObsSpeed < this->minOutsideFovAvgObsSpeed_){
                return false;
            }

            if (avgObsSpeed > this->maxOutsideFovAvgObsSpeed_){
                return false;
            }

            if (!this->isNaturalMotion(trackIdx, currDetectedBBox)){
                return false;
            }

            if (!this->passesVelocityDirectionGate(trackIdx, currDetectedBBox, false, false)){
                return false;
            }

            return true;
        }

        // ============================================================
        // COMPORTAMENTO ORIGINALE DENTRO FOV / CASI NON BOOTSTRAP
        // ============================================================

        if (newHitStreak < this->minConfirmHits_){
            return false;
        }

        if (!this->trackSteadyObjects_ && obsSpeed < this->stationarySpeedThresh_){
            return false;
        }

        if (!this->passesVelocityDirectionGate(trackIdx, currDetectedBBox, false, false)){
            return false;
        }

        return true;
    }

    bool dynamicDetector::isAbruptTurnAllowedOutsideFov(
        int trackIdx,
        const joDetector::box3D& predictedBox,
        const joDetector::box3D& currentBox,
        double dt) const
    {
        if (trackIdx < 0 ||
            trackIdx >= static_cast<int>(this->boxHist_.size()) ||
            this->boxHist_[trackIdx].empty())
        {
            return false;
        }

        // NEW: abrupt turn solo per track già abbastanza mature
        const int trackAge =
            (trackIdx < static_cast<int>(this->trackAge_.size()))
                ? this->trackAge_[trackIdx]
                : 0;

        if (trackAge < this->outsideFovTurnMinTrackAge_){
            return false;
        }

        const Eigen::Vector3d predPos(predictedBox.x, predictedBox.y, predictedBox.z);
        const Eigen::Vector3d currPos(currentBox.x, currentBox.y, currentBox.z);

        const bool predOutsideFov = !this->isInCameraFOV(predPos);
        const bool currOutsideFov = !this->isInCameraFOV(currPos);

        if (!predOutsideFov || !currOutsideFov){
            return false;
        }

        // Non tocchiamo YOLO o track con storia YOLO
        if (predictedBox.is_yolo_candidate || currentBox.is_yolo_candidate){
            return false;
        }

        if (this->boxHist_[trackIdx][0].is_yolo_candidate){
            return false;
        }

        dt = this->clampPositive(dt, 1e-3);

        const joDetector::box3D& prevBox = this->boxHist_[trackIdx].front();

        const double dxObs = currentBox.x - prevBox.x;
        const double dyObs = currentBox.y - prevBox.y;
        const double obsDist = std::sqrt(dxObs * dxObs + dyObs * dyObs);
        const double obsSpeed = obsDist / dt;

        if (obsSpeed < this->outsideFovTurnMinObsSpeed_){
            return false;
        }

        if (obsSpeed > this->outsideFovTurnMaxSpeed_){
            return false;
        }

        const double dxPred = currentBox.x - predictedBox.x;
        const double dyPred = currentBox.y - predictedBox.y;
        const double predDist = std::sqrt(dxPred * dxPred + dyPred * dyPred);

        if (predDist > this->outsideFovTurnMaxPosDist_){
            return false;
        }

        const double relSizeDiff =
            this->computeRelativeSizeDiff(predictedBox, currentBox);

        if (relSizeDiff > this->outsideFovTurnMaxSizeDiff_){
            return false;
        }

        // NEW: non accettare abrupt turn se non c'è moto osservato coerente
        double netDisp = 0.0;
        double netSpeed = 0.0;
        double pathLen = 0.0;
        double straightness = 0.0;
        double maxStepSpeed = 0.0;

        const bool observedMotionOk =
            this->hasConsistentObservedMotionOutsideFov(
                trackIdx,
                this->outsideFovClassWindow_,
                netDisp,
                netSpeed,
                pathLen,
                straightness,
                maxStepSpeed);

        if (!observedMotionOk){
            return false;
        }

        return true;
    } 

    bool dynamicDetector::hasConsistentObservedMotionOutsideFov(
        int trackIdx,
        int window,
        double& netDisp,
        double& netSpeed,
        double& pathLen,
        double& straightness,
        double& maxStepSpeed) const
    {
        netDisp = 0.0;
        netSpeed = 0.0;
        pathLen = 0.0;
        straightness = 0.0;
        maxStepSpeed = 0.0;

        if (trackIdx < 0 ||
            trackIdx >= static_cast<int>(this->boxHist_.size()) ||
            this->boxHist_[trackIdx].empty())
        {
            return false;
        }

        const int histSize = static_cast<int>(this->boxHist_[trackIdx].size());

        if (histSize < 2){
            return false;
        }

        const int k = std::min(window, histSize - 1);

        if (k < 1){
            return false;
        }

        const double dt = this->clampPositive(this->dt_, 1e-3);

        const auto& curr = this->boxHist_[trackIdx][0];
        const auto& past = this->boxHist_[trackIdx][k];

        const double dxNet = curr.x - past.x;
        const double dyNet = curr.y - past.y;

        netDisp = std::sqrt(dxNet * dxNet + dyNet * dyNet);
        netSpeed = netDisp / this->clampPositive(dt * static_cast<double>(k), 1e-3);

        for (int i = 0; i < k; ++i){
            const auto& a = this->boxHist_[trackIdx][i];
            const auto& b = this->boxHist_[trackIdx][i + 1];

            const double dx = a.x - b.x;
            const double dy = a.y - b.y;

            const double stepDist = std::sqrt(dx * dx + dy * dy);
            const double stepSpeed = stepDist / dt;

            pathLen += stepDist;
            maxStepSpeed = std::max(maxStepSpeed, stepSpeed);
        }

        straightness = netDisp / this->clampPositive(pathLen, 1e-6);

        if (netDisp < this->outsideFovClassMinNetDisp_){
            return false;
        }

        if (netSpeed < this->outsideFovClassMinNetSpeed_){
            return false;
        }

        if (straightness < this->outsideFovClassMinStraightness_){
            return false;
        }

        if (maxStepSpeed > this->outsideFovClassMaxStepSpeed_){
            return false;
        }

        return true;
    }

    void dynamicDetector::boxAssociation(std::vector<int>& bestMatch){
        int numObjs = int(this->filteredBBoxes_.size());

        if (this->boxHist_.size() == 0){
            this->boxHist_.resize(numObjs);
            this->pcHist_.resize(numObjs);
            this->pcCenterHist_.resize(numObjs);
            this->missedFrames_.resize(numObjs, 0);
            this->hitStreak_.resize(numObjs, 1);
            this->trackAge_.resize(numObjs, 1);
            this->confirmedTracks_.resize(numObjs, false);
            this->staticStreak_.resize(numObjs, 0);

            bestMatch.resize(this->filteredBBoxes_.size(), -1);

            for (int i = 0; i < numObjs; ++i){
                this->filteredBBoxes_[i].id = this->nextTrackId_;
                this->nextTrackId_++;

                this->boxHist_[i].push_back(this->filteredBBoxes_[i]);
                this->pcHist_[i].push_back(this->filteredPcClusters_[i]);
                this->pcCenterHist_[i].push_back(this->filteredPcClusterCenters_[i]);

                MatrixXd states, A, B, H, P, Q, R;
                this->kalmanFilterMatrixAccV2(this->filteredBBoxes_[i], states, A, B, H, P, Q, R);

                joDetector::kalman_filter newFilter;
                newFilter.setup(states, A, B, H, P, Q, R);
                this->filters_.push_back(newFilter);

                // No immediate confirmation: minConfirmHits_ consecutive hits are
                // required before any track enters the output, even for YOLO detections.
            }
        }
        else{
            if (this->newDetectFlag_){
                this->boxAssociationHelper(bestMatch);
            }
            else{
                bestMatch.clear();
            }
        }

        this->newDetectFlag_ = false;
    }


    void dynamicDetector::boxAssociationHelper(std::vector<int>& bestMatch)
    {
        std::vector<joDetector::box3D> predictedBBoxes;
        std::vector<Eigen::Vector3d> predictedPcCenters;
        std::vector<joDetector::box3D> previousObservedBBoxes;

        this->getPredictedBBoxesFromFilters(predictedBBoxes, predictedPcCenters);

        this->getPreviousObservedBBoxes(previousObservedBBoxes);

        this->findBestMatch(predictedBBoxes, previousObservedBBoxes, bestMatch);
    }

    void dynamicDetector::getPreviousObservedBBoxes(std::vector<joDetector::box3D>& previousObservedBBoxes) const
    {
        previousObservedBBoxes.clear();
        previousObservedBBoxes.reserve(this->boxHist_.size());

        for (size_t i = 0; i < this->boxHist_.size(); ++i){
            if (this->boxHist_[i].empty()){
                continue;
            }
            previousObservedBBoxes.push_back(this->boxHist_[i][0]);
        }
    }

    void dynamicDetector::findBestMatch(const std::vector<joDetector::box3D>& predictedBBoxes,
                                        const std::vector<joDetector::box3D>& previousObservedBBoxes,
                                        std::vector<int>& bestMatch)
    {
        const int numObjs = static_cast<int>(this->filteredBBoxes_.size());
        bestMatch.assign(numObjs, -1);

        std::vector<joDetector::matchCandidate> candidates;
        candidates.reserve(numObjs * std::max<size_t>(1, predictedBBoxes.size()));

        const double dt = this->clampPositive(this->dt_, 1e-3);

        for (int i = 0; i < numObjs; ++i){
            const joDetector::box3D& currBBox = this->filteredBBoxes_[i];

            for (size_t j = 0; j < predictedBBoxes.size(); ++j){
                if (j >= previousObservedBBoxes.size()){
                    continue;
                }

                const joDetector::box3D& predBBox = predictedBBoxes[j];
                const joDetector::box3D& prevObsBBox = previousObservedBBoxes[j];

                double predPosDist = 0.0;
                double requiredSpeed = 0.0;
                double relSizeDiff = 0.0;
                double predIou2d = 0.0;
                double prevObsPosDist = 0.0;
                double prevObsIou2d = 0.0;
                std::string rejectReason;

                const double score = this->computeAssociationScore(predBBox,
                                                                prevObsBBox,
                                                                currBBox,
                                                                dt,
                                                                predPosDist,
                                                                requiredSpeed,
                                                                relSizeDiff,
                                                                predIou2d,
                                                                prevObsPosDist,
                                                                prevObsIou2d,
                                                                rejectReason);

                int trackId = -1;
                if (j < this->boxHist_.size() && !this->boxHist_[j].empty()){
                    trackId = this->boxHist_[j][0].id;
                }

                if (!rejectReason.empty()){
                    if (this->enableTrackingDebugLogs_){
                        RCLCPP_DEBUG(
                            this->nh_->get_logger(),
                            "[TRACK_%s] curr=%d prev=%d track_id=%d predPosDist=%.3f reqSpeed=%.3f relSizeDiff=%.3f predIou2d=%.3f prevObsPosDist=%.3f prevObsIou2d=%.3f score=%.3f",
                            rejectReason.c_str(),
                            i,
                            static_cast<int>(j),
                            trackId,
                            predPosDist,
                            requiredSpeed,
                            relSizeDiff,
                            predIou2d,
                            prevObsPosDist,
                            prevObsIou2d,
                            score);
                    }
                    continue;
                }

                if (this->enableTrackingDebugLogs_){
                    RCLCPP_DEBUG(
                        this->nh_->get_logger(),
                        "[TRACK_CANDIDATE] curr=%d prev=%d track_id=%d predPosDist=%.3f reqSpeed=%.3f relSizeDiff=%.3f predIou2d=%.3f prevObsPosDist=%.3f prevObsIou2d=%.3f score=%.3f",
                        i,
                        static_cast<int>(j),
                        trackId,
                        predPosDist,
                        requiredSpeed,
                        relSizeDiff,
                        predIou2d,
                        prevObsPosDist,
                        prevObsIou2d,
                        score);
                }

                joDetector::matchCandidate candidate;
                candidate.currIdx = i;
                candidate.prevIdx = static_cast<int>(j);
                candidate.score = score;
                candidates.push_back(candidate);
            }
        }

        std::sort(candidates.begin(), candidates.end(),
                [](const joDetector::matchCandidate& a, const joDetector::matchCandidate& b){
                    if (a.currIdx == b.currIdx){
                        return a.score > b.score;
                    }
                    return a.score > b.score;
                });

        this->assignMatchesGlobally(candidates,
                                    numObjs,
                                    static_cast<int>(predictedBBoxes.size()),
                                    bestMatch);

        if (this->enableTrackingDebugLogs_){
            for (int currIdx = 0; currIdx < numObjs; ++currIdx){
                const int prevIdx = bestMatch[currIdx];
                int trackId = -1;
                if (prevIdx >= 0 && prevIdx < static_cast<int>(this->boxHist_.size()) && !this->boxHist_[prevIdx].empty()){
                    trackId = this->boxHist_[prevIdx][0].id;
                }

                RCLCPP_DEBUG(
                    this->nh_->get_logger(),
                    "[TRACK_ASSIGN_GLOBAL] curr=%d prev=%d track_id=%d",
                    currIdx,
                    prevIdx,
                    trackId);
            }
        }
    }

    void dynamicDetector::kalmanFilterAndUpdateHist(const std::vector<int>& bestMatch){
        std::vector<std::deque<joDetector::box3D>> boxHistTemp;
        std::vector<std::deque<std::vector<Eigen::Vector3d>>> pcHistTemp;
        std::vector<std::deque<Eigen::Vector3d>> pcCenterHistTemp;
        std::vector<joDetector::kalman_filter> filtersTemp;
        std::vector<int> missedFramesTemp;
        std::vector<joDetector::box3D> trackedBBoxesTemp;

        std::vector<int> hitStreakTemp;
        std::vector<int> trackAgeTemp;
        std::vector<bool> confirmedTracksTemp;
        std::vector<int> staticStreakTemp;
        std::vector<int> nonYoloInFovStreakTemp;
        std::vector<double> yoloXWidthTemp;
        std::vector<double> yoloYWidthTemp;
        std::vector<int> yoloBaseMismatchStreakTemp;

        std::deque<joDetector::box3D> newSingleBoxHist;
        std::deque<std::vector<Eigen::Vector3d>> newSinglePcHist;
        std::deque<Eigen::Vector3d> newSinglePcCenterHist;
        joDetector::kalman_filter newFilter;

        int numObjs = static_cast<int>(this->filteredBBoxes_.size());
        int prevTracks = static_cast<int>(this->boxHist_.size());

        std::vector<bool> prevMatched(prevTracks, false);

        // 1) Track matchate
        for (int i = 0; i < numObjs; ++i){
            if (i >= static_cast<int>(bestMatch.size()) || bestMatch[i] < 0){
                continue;
            }

            int matchIdx = bestMatch[i];
            prevMatched[matchIdx] = true;

            boxHistTemp.push_back(this->boxHist_[matchIdx]);
            pcHistTemp.push_back(this->pcHist_[matchIdx]);
            pcCenterHistTemp.push_back(this->pcCenterHist_[matchIdx]);
            filtersTemp.push_back(this->filters_[matchIdx]);
            missedFramesTemp.push_back(0);

            joDetector::box3D currDetectedBBox = this->filteredBBoxes_[i];
            joDetector::box3D newEstimatedBBox;

            Eigen::MatrixXd Z;
            this->getKalmanObservationAccV2(currDetectedBBox, Z);
            filtersTemp.back().estimate(Z, MatrixXd::Zero(6,1));

            newEstimatedBBox.x = filtersTemp.back().output(0);
            newEstimatedBBox.y = filtersTemp.back().output(1);
            newEstimatedBBox.z = currDetectedBBox.z;
            newEstimatedBBox.Vx = filtersTemp.back().output(2);
            newEstimatedBBox.Vy = filtersTemp.back().output(3);
            newEstimatedBBox.Ax = filtersTemp.back().output(4);
            newEstimatedBBox.Ay = filtersTemp.back().output(5);

            newEstimatedBBox.x_width = currDetectedBBox.x_width;
            newEstimatedBBox.y_width = currDetectedBBox.y_width;
            newEstimatedBBox.z_width = currDetectedBBox.z_width;
            // Sticky flags: is_yolo_candidate propagates forward so CASE I in
            // classificationCB keeps firing even outside the camera FOV.
            // is_dynamic is NOT propagated: classificationCB decides fresh each frame.
            const bool prevYoloCand          = !this->boxHist_[matchIdx].empty() && this->boxHist_[matchIdx][0].is_yolo_candidate;
            const bool prevDynamic           = !this->boxHist_[matchIdx].empty() && this->boxHist_[matchIdx][0].is_dynamic;
            const bool prevPotentiallyDynamic = !this->boxHist_[matchIdx].empty() && this->boxHist_[matchIdx][0].is_potentially_dynamic;
            // is_yolo_candidate: sticky from YOLO detector (sensor output, track-level).
            newEstimatedBBox.is_yolo_candidate = currDetectedBBox.is_yolo_candidate || prevYoloCand;

            // YOLO decay (FOV-based): if the track is inside the camera FOV and the current
            // detection is NOT a YOLO detection, increment a counter.
            const Eigen::Vector3d bboxPos(newEstimatedBBox.x, newEstimatedBBox.y, newEstimatedBBox.z);
            const bool insideFov = this->isInCameraFOV(bboxPos);
            const int oldNonYoloStreak =
                (matchIdx < static_cast<int>(this->nonYoloInFovStreak_.size())) ? this->nonYoloInFovStreak_[matchIdx] : 0;
            int newNonYoloStreak = 0;
            if (newEstimatedBBox.is_yolo_candidate && insideFov && !currDetectedBBox.is_yolo_candidate){
                newNonYoloStreak = oldNonYoloStreak + 1;
            }

            if (newNonYoloStreak >= this->maxNonYoloInFovFrames_){
                newEstimatedBBox.is_yolo_candidate = false;
                newNonYoloStreak = 0;
            }
            nonYoloInFovStreakTemp.push_back(newNonYoloStreak);

            // YOLO decay (base size mismatch): ONLY outside FOV.
            // Compare the current detection's base dimensions against the last
            // YOLO detection's dims. If size differs too much for several
            // consecutive frames, this is likely an ID swap.
            double XW = (matchIdx < static_cast<int>(this->yoloXWidth_.size())) ? this->yoloXWidth_[matchIdx] : 0.0;
            double YW = (matchIdx < static_cast<int>(this->yoloYWidth_.size())) ? this->yoloYWidth_[matchIdx] : 0.0;
            int oldBaseMismatch = (matchIdx < static_cast<int>(this->yoloBaseMismatchStreak_.size())) ? this->yoloBaseMismatchStreak_[matchIdx] : 0;

            // Update stored dims on a fresh YOLO detection: keep the maximum seen so far.
            // Using the max prevents the baseline from shrinking (e.g. partial occlusion),
            // so the ID-swap growth check is always relative to the largest known size.
            if (currDetectedBBox.is_yolo_candidate){
                XW = std::max(XW, currDetectedBBox.x_width);
                YW = std::max(YW, currDetectedBBox.y_width);
            }

            int newBaseMismatch = 0;
            if (!insideFov && newEstimatedBBox.is_yolo_candidate && XW > 0.0 && YW > 0.0){
                // Outside FOV the bbox naturally shrinks (LiDAR-only is sparser than LiDAR+depth).
                // Only flag growth: a bbox growing significantly vs. the YOLO baseline indicates
                // an ID swap toward a larger object (wall, vehicle, noisy cluster).
                // Shrinkage is ignored — it is a physical artifact, not an ID swap.
                const double growthX = (currDetectedBBox.x_width - XW) / std::max(XW, 0.01);
                const double growthY = (currDetectedBBox.y_width - YW) / std::max(YW, 0.01);
                if (growthX > this->yoloBaseMismatchThresh_ || growthY > this->yoloBaseMismatchThresh_){
                    newBaseMismatch = oldBaseMismatch + 1;
                }
                // else: reset to 0 (dimensions within expected range → legitimate track)
            }

            if (newBaseMismatch >= this->maxYoloBaseMismatchFrames_){
                newEstimatedBBox.is_yolo_candidate = false;
                newBaseMismatch = 0;
                XW = 0.0;
                YW = 0.0;
            }
            yoloXWidthTemp.push_back(XW);
            yoloYWidthTemp.push_back(YW);
            yoloBaseMismatchStreakTemp.push_back(newBaseMismatch);

            // is_dynamic: starts false; classificationCB will set it if appropriate.
            newEstimatedBBox.is_dynamic = false;
            newEstimatedBBox.id = this->boxHist_[matchIdx][0].id;

            if (int(boxHistTemp.back().size()) == this->histSize_){
                boxHistTemp.back().pop_back();
                pcHistTemp.back().pop_back();
                pcCenterHistTemp.back().pop_back();
            }

            boxHistTemp.back().push_front(newEstimatedBBox);
            pcHistTemp.back().push_front(this->filteredPcClusters_[i]);
            pcCenterHistTemp.back().push_front(this->filteredPcClusterCenters_[i]);

            const int oldHitStreak =
                (matchIdx < static_cast<int>(this->hitStreak_.size())) ? this->hitStreak_[matchIdx] : 0;
            const int oldTrackAge =
                (matchIdx < static_cast<int>(this->trackAge_.size())) ? this->trackAge_[matchIdx] : 0;
            const bool oldConfirmed =
                (matchIdx < static_cast<int>(this->confirmedTracks_.size())) ? this->confirmedTracks_[matchIdx] : false;

            const int newHitStreak = oldHitStreak + 1;
            const int newTrackAge = oldTrackAge + 1;

            staticStreakTemp.push_back(0); // unused, kept for vector size consistency

            // Once confirmed a track stays confirmed until it dies by maxMissedFrames_.
            // There is no active de-confirmation: a person who stops walking stays tracked.
            const bool newConfirmed = oldConfirmed || this->shouldConfirmTrack(matchIdx, currDetectedBBox, newHitStreak);

            hitStreakTemp.push_back(newHitStreak);
            trackAgeTemp.push_back(newTrackAge);
            confirmedTracksTemp.push_back(newConfirmed);

            if (newConfirmed){
                // newEstimatedBBox.is_yolo_candidate carries TRACK-level YOLO evidence:
                //   is_yolo_candidate = currDetectedBBox.is_yolo_candidate || prevYoloCand
                // where prevYoloCand is read from boxHist_ (previous frame's sticky).
                // This means it is true if YOLO fired on this track in ANY past frame,
                // regardless of whether YOLO is firing this frame (YOLO runs slower).
                // currDetectedBBox.is_yolo_candidate alone would be false on frames where
                // YOLO hasn't updated — using newEstimatedBBox here is intentional.

                // Suppress stationary non-YOLO tracks from the output topic.
                // A confirmed non-YOLO track that has stopped moving is kept alive
                // internally (confirmedTracks_ remains true) so it can quickly re-enter
                // the output when it moves again, but it is not published.
                // Tracks with YOLO evidence (current or sticky history) are always published
                // even when stationary — they represent known dynamic objects (people, vehicles).
                const double kfSpeed = std::sqrt(newEstimatedBBox.Vx * newEstimatedBBox.Vx +
                                                  newEstimatedBBox.Vy * newEstimatedBBox.Vy);
                const bool suppressStationary = !this->trackSteadyObjects_ &&
                                                !newEstimatedBBox.is_yolo_candidate &&
                                                (kfSpeed < this->stationarySpeedThresh_);

                if (!suppressStationary){
                    // is_yolo_candidate is taken from newEstimatedBBox (track-level sticky).
                    // is_dynamic: use previous frame's classification result (1-frame delay)
                    // since classificationCB runs after tracking.
                    joDetector::box3D outputBBox = currDetectedBBox;
                    outputBBox.Vx               = newEstimatedBBox.Vx;
                    outputBBox.Vy               = newEstimatedBBox.Vy;
                    outputBBox.Ax               = newEstimatedBBox.Ax;
                    outputBBox.Ay               = newEstimatedBBox.Ay;
                    outputBBox.id               = newEstimatedBBox.id;
                    outputBBox.is_yolo_candidate     = newEstimatedBBox.is_yolo_candidate;
                    outputBBox.is_dynamic             = prevDynamic;
                    outputBBox.is_potentially_dynamic = prevPotentiallyDynamic;
                    outputBBox.track_age              = static_cast<uint32_t>(newTrackAge);
                    trackedBBoxesTemp.push_back(outputBBox);
                }
            }

            if (this->enableTrackingDebugLogs_){
                RCLCPP_DEBUG(
                    this->nh_->get_logger(),
                    "[TRACK_UPDATE_MATCHED] track_id=%d x=%.3f y=%.3f vx=%.3f vy=%.3f hit=%d age=%d confirmed=%d",
                    static_cast<int>(newEstimatedBBox.id),
                    newEstimatedBBox.x,
                    newEstimatedBBox.y,
                    newEstimatedBBox.Vx,
                    newEstimatedBBox.Vy,
                    newHitStreak,
                    newTrackAge,
                    newConfirmed ? 1 : 0);
            }
        }

        // 2) Track non matchate: propagazione interna, mai in output
        for (int j = 0; j < prevTracks; ++j){
            if (prevMatched[j]){
                continue;
            }

            int newMissed = this->missedFrames_[j] + 1;
            // YOLO-confirmed tracks survive longer: they represent known dynamic objects
            // (persons, vehicles) that may temporarily leave the FOV or be occluded.
            const bool trackIsYolo = !this->boxHist_[j].empty() && this->boxHist_[j][0].is_yolo_candidate;
            const int effectiveMaxMissed = trackIsYolo ? this->maxMissedFramesYolo_ : this->maxMissedFrames_;
            if (newMissed > effectiveMaxMissed){
                if (this->enableTrackingDebugLogs_){
                    int oldTrackId = (!this->boxHist_[j].empty()) ? static_cast<int>(this->boxHist_[j][0].id) : -1;
                    RCLCPP_DEBUG(
                        this->nh_->get_logger(),
                        "[TRACK_DROP_MISSED] track_id=%d missed=%d maxMissed=%d yolo=%d",
                        oldTrackId,
                        newMissed,
                        effectiveMaxMissed,
                        trackIsYolo ? 1 : 0);
                }
                continue;
            }

            std::deque<joDetector::box3D> hist = this->boxHist_[j];
            std::deque<std::vector<Eigen::Vector3d>> pcHist = this->pcHist_[j];
            std::deque<Eigen::Vector3d> pcCenterHist = this->pcCenterHist_[j];
            joDetector::kalman_filter filter = this->filters_[j];

            filter.predict(MatrixXd::Zero(6,1));

            joDetector::box3D predictedBBox = hist.front();
            predictedBBox.x = filter.output(0);
            predictedBBox.y = filter.output(1);
            predictedBBox.Vx = filter.output(2);
            predictedBBox.Vy = filter.output(3);
            predictedBBox.Ax = filter.output(4);
            predictedBBox.Ay = filter.output(5);
            predictedBBox.id = hist.front().id;
            // is_dynamic not propagated: classificationCB decides fresh each frame.
            predictedBBox.is_dynamic = false;

            if (int(hist.size()) == this->histSize_){
                hist.pop_back();
                pcHist.pop_back();
                pcCenterHist.pop_back();
            }

            hist.push_front(predictedBBox);

            Eigen::Vector3d predictedPcCenter = pcCenterHist.front();
            predictedPcCenter(0) = predictedBBox.x;
            predictedPcCenter(1) = predictedBBox.y;
            pcCenterHist.push_front(predictedPcCenter);

            if (!pcHist.empty()){
                pcHist.push_front(pcHist.front());
            }

            boxHistTemp.push_back(hist);
            pcHistTemp.push_back(pcHist);
            pcCenterHistTemp.push_back(pcCenterHist);
            filtersTemp.push_back(filter);
            missedFramesTemp.push_back(newMissed);

            const int oldTrackAge =
                (j < static_cast<int>(this->trackAge_.size())) ? this->trackAge_[j] : 0;
            const bool oldConfirmed =
                (j < static_cast<int>(this->confirmedTracks_.size())) ? this->confirmedTracks_[j] : false;

            const int oldStaticStreakMissed =
                (j < static_cast<int>(this->staticStreak_.size())) ? this->staticStreak_[j] : 0;
            staticStreakTemp.push_back(oldStaticStreakMissed); // no detection → keep existing streak

            const int oldNonYoloStreakMissed =
                (j < static_cast<int>(this->nonYoloInFovStreak_.size())) ? this->nonYoloInFovStreak_[j] : 0;
            nonYoloInFovStreakTemp.push_back(oldNonYoloStreakMissed); // no detection → keep existing streak

            const double oldXW = (j < static_cast<int>(this->yoloXWidth_.size())) ? this->yoloXWidth_[j] : 0.0;
            const double oldYW = (j < static_cast<int>(this->yoloYWidth_.size())) ? this->yoloYWidth_[j] : 0.0;
            const int oldBaseMismatchMissed = (j < static_cast<int>(this->yoloBaseMismatchStreak_.size())) ? this->yoloBaseMismatchStreak_[j] : 0;
            yoloXWidthTemp.push_back(oldXW);
            yoloYWidthTemp.push_back(oldYW);
            yoloBaseMismatchStreakTemp.push_back(oldBaseMismatchMissed); // no detection → keep existing streak

            hitStreakTemp.push_back(0);
            trackAgeTemp.push_back(oldTrackAge + 1);
            confirmedTracksTemp.push_back(oldConfirmed);

            // Unmatched tracks are kept alive internally (boxHistTemp, filtersTemp) so
            // they can re-associate with future detections, but are never pushed to
            // trackedBBoxesTemp: no Kalman-predicted bbox ever appears in the output.

            if (this->enableTrackingDebugLogs_){
                RCLCPP_DEBUG(
                    this->nh_->get_logger(),
                    "[TRACK_PROPAGATE_INTERNAL] track_id=%d x=%.3f y=%.3f vx=%.3f vy=%.3f missed=%d confirmed=%d",
                    static_cast<int>(predictedBBox.id),
                    predictedBBox.x,
                    predictedBBox.y,
                    predictedBBox.Vx,
                    predictedBBox.Vy,
                    newMissed,
                    oldConfirmed ? 1 : 0);
            }
        }

        // 3) Detection non matchate: nascono tentative, vanno in output solo se già dinamiche/umane
        for (int i = 0; i < numObjs; ++i){
            if (i < static_cast<int>(bestMatch.size()) && bestMatch[i] >= 0){
                continue;
            }

            const joDetector::box3D& currDetectedBBox = this->filteredBBoxes_[i];
            const Eigen::Vector3d& currPcCenter = this->filteredPcClusterCenters_[i];

            if (this->isCloseToExistingTrack(currDetectedBBox, currPcCenter, prevMatched)){
                continue;
            }

            int duplicateTrackId = -1;
            if (this->isDuplicateOfTrackedThisFrame(currDetectedBBox, trackedBBoxesTemp, duplicateTrackId)){
                if (this->enableTrackingDebugLogs_){
                    RCLCPP_DEBUG(
                        this->nh_->get_logger(),
                        "[TRACK_NEW_BLOCKED_BY_CURRENT_FRAME] det=%d duplicate_of_track_id=%d",
                        i,
                        duplicateTrackId);
                }
                continue;
            }

            boxHistTemp.push_back(newSingleBoxHist);
            pcHistTemp.push_back(newSinglePcHist);
            pcCenterHistTemp.push_back(newSinglePcCenterHist);

            joDetector::box3D newEstimatedBBox = currDetectedBBox;

            Eigen::MatrixXd states, A, B, H, P, Q, R;
            this->kalmanFilterMatrixAccV2(currDetectedBBox, states, A, B, H, P, Q, R);

            newFilter.setup(states, A, B, H, P, Q, R);
            filtersTemp.push_back(newFilter);
            missedFramesTemp.push_back(0);

            newEstimatedBBox.id = this->nextTrackId_;
            this->nextTrackId_++;

            boxHistTemp.back().push_front(newEstimatedBBox);
            pcHistTemp.back().push_front(this->filteredPcClusters_[i]);
            pcCenterHistTemp.back().push_front(this->filteredPcClusterCenters_[i]);

            hitStreakTemp.push_back(1);
            trackAgeTemp.push_back(1);
            staticStreakTemp.push_back(0); // new track, no static history
            nonYoloInFovStreakTemp.push_back(0); // new track, no YOLO decay history
            // Store original YOLO base dimensions if this is a YOLO detection.
            if (currDetectedBBox.is_yolo_candidate){
                yoloXWidthTemp.push_back(currDetectedBBox.x_width);
                yoloYWidthTemp.push_back(currDetectedBBox.y_width);
            } else {
                yoloXWidthTemp.push_back(0.0);
                yoloYWidthTemp.push_back(0.0);
            }
            yoloBaseMismatchStreakTemp.push_back(0);

            // YOLO detections are confirmed immediately on their first frame.
            // Non-YOLO detections start tentative and need minConfirmHits_ to confirm.
            const bool immediateConfirm = currDetectedBBox.is_yolo_candidate;
            confirmedTracksTemp.push_back(immediateConfirm);

            if (immediateConfirm){
                joDetector::box3D outputBBox = currDetectedBBox;
                outputBBox.id        = newEstimatedBBox.id;
                outputBBox.track_age = 1;
                trackedBBoxesTemp.push_back(outputBBox);
            }

            if (this->enableTrackingDebugLogs_){
                RCLCPP_INFO(
                    this->nh_->get_logger(),
                    "[TRACK_NEW] det=%d new_track_id=%d x=%.3f y=%.3f z=%.3f confirmed=%d",
                    i,
                    static_cast<int>(newEstimatedBBox.id),
                    newEstimatedBBox.x,
                    newEstimatedBBox.y,
                    newEstimatedBBox.z,
                    immediateConfirm ? 1 : 0);
            }
        }

        // 4) Tracked bbox dimensions: always taken directly from the current detection
        // (outputBBox = currDetectedBBox). No history-based dimension override is applied
        // because any lock-to-prev logic creates a self-reinforcing loop when a bad merged
        // detection enters history — the bad dimension propagates forward indefinitely and
        // the output diverges from the filtered bbox. The detection pipeline (merging,
        // DBSCAN, UV filter) is the authoritative source for bbox dimensions.

        this->boxHist_ = boxHistTemp;
        this->pcHist_ = pcHistTemp;
        this->pcCenterHist_ = pcCenterHistTemp;
        this->filters_ = filtersTemp;
        this->missedFrames_ = missedFramesTemp;
        this->trackedBBoxes_ = trackedBBoxesTemp;
        this->hitStreak_ = hitStreakTemp;
        this->trackAge_ = trackAgeTemp;
        this->confirmedTracks_ = confirmedTracksTemp;
        this->staticStreak_ = staticStreakTemp;
        this->nonYoloInFovStreak_ = nonYoloInFovStreakTemp;
        this->yoloXWidth_ = yoloXWidthTemp;
        this->yoloYWidth_ = yoloYWidthTemp;
        this->yoloBaseMismatchStreak_ = yoloBaseMismatchStreakTemp;
    }

    // ============================================================================
    // Kalman Filter V2: position-only observation model
    // The state vector is the same 6D [x, y, vx, vy, ax, ay] with the same
    // constant-acceleration transition matrix A.
    // The key difference: H is 2x6 (observe only position), so the filter
    // internally estimates velocity and acceleration from position updates alone.
    // This avoids double-counting noisy finite-difference velocity/acceleration
    // as direct observations.
    // ============================================================================

    void dynamicDetector::kalmanFilterMatrixAccV2(const joDetector::box3D& currDetectedBBox,
                                                  MatrixXd& states,
                                                  MatrixXd& A,
                                                  MatrixXd& B,
                                                  MatrixXd& H,
                                                  MatrixXd& P,
                                                  MatrixXd& Q,
                                                  MatrixXd& R)
    {
        const double dt = this->clampPositive(this->dt_, 1e-3);
        const double dt2 = dt * dt;

        states.resize(6, 1);
        states(0) = currDetectedBBox.x;
        states(1) = currDetectedBBox.y;
        states(2) = 0.0;
        states(3) = 0.0;
        states(4) = 0.0;
        states(5) = 0.0;

        A.resize(6, 6);
        A << 1, 0, dt, 0, 0.5 * dt2, 0,
             0, 1, 0, dt, 0, 0.5 * dt2,
             0, 0, 1,  0, dt,         0,
             0, 0, 0,  1, 0,          dt,
             0, 0, 0,  0, 1,          0,
             0, 0, 0,  0, 0,          1;

        B = MatrixXd::Zero(6, 6);

        // Observe only position (2x6)
        H = MatrixXd::Zero(2, 6);
        H(0, 0) = 1.0;
        H(1, 1) = 1.0;

        P = MatrixXd::Identity(6, 6) * this->eP_v2_;

        Q = MatrixXd::Zero(6, 6);
        Q(0, 0) = this->eQPos_v2_;
        Q(1, 1) = this->eQPos_v2_;
        Q(2, 2) = this->eQVel_v2_;
        Q(3, 3) = this->eQVel_v2_;
        Q(4, 4) = this->eQAcc_v2_;
        Q(5, 5) = this->eQAcc_v2_;

        // Observation noise: only 2x2 for position
        R = MatrixXd::Zero(2, 2);
        R(0, 0) = this->eRPos_v2_;
        R(1, 1) = this->eRPos_v2_;
    }

    void dynamicDetector::getKalmanObservationAccV2(const joDetector::box3D& currDetectedBBox,
                                                     MatrixXd& Z)
    {
        Z = MatrixXd::Zero(2, 1);
        Z(0) = currDetectedBBox.x;
        Z(1) = currDetectedBBox.y;
    }

    void dynamicDetector::getPredictedBBoxesFromFilters(std::vector<joDetector::box3D>& propedBBoxes,
                                                        std::vector<Eigen::Vector3d>& propedPcCenters){
        propedBBoxes.clear();
        propedPcCenters.clear();

        for (size_t i = 0; i < this->boxHist_.size(); ++i){
            if (this->boxHist_[i].empty()){
                continue;
            }

            joDetector::kalman_filter predictedFilter = this->filters_[i];
            predictedFilter.predict(MatrixXd::Zero(6,1));

            joDetector::box3D predBBox = this->boxHist_[i][0];
            predBBox.x = predictedFilter.output(0);
            predBBox.y = predictedFilter.output(1);
            predBBox.Vx = predictedFilter.output(2);
            predBBox.Vy = predictedFilter.output(3);
            predBBox.Ax = predictedFilter.output(4);
            predBBox.Ay = predictedFilter.output(5);

            Eigen::Vector3d predPcCenter = this->pcCenterHist_[i][0];
            predPcCenter(0) = predBBox.x;
            predPcCenter(1) = predBBox.y;

            propedBBoxes.push_back(predBBox);
            propedPcCenters.push_back(predPcCenter);
        }
    }

    void dynamicDetector::populatePredictionVisualizationBBoxes(){
        this->predictedBBoxesActive_.clear();
        this->predictedBBoxesMissed_.clear();
        this->predictedBBoxesUnconfirmed_.clear();

        const int numSteps = this->predictionVisualizationSteps_ < 1 ? 1 : this->predictionVisualizationSteps_;

        for (size_t i = 0; i < this->boxHist_.size(); ++i){
            if (this->boxHist_[i].empty() || i >= this->filters_.size()){
                continue;
            }

            const bool confirmed = (i < this->confirmedTracks_.size()) && this->confirmedTracks_[i];
            const int missed = (i < this->missedFrames_.size()) ? this->missedFrames_[i] : 0;

            joDetector::kalman_filter predictedFilter = this->filters_[i];
            for (int step = 0; step < numSteps; ++step){
                predictedFilter.predict(MatrixXd::Zero(6,1));

                joDetector::box3D predBBox = this->boxHist_[i][0];
                predBBox.x = predictedFilter.output(0);
                predBBox.y = predictedFilter.output(1);
                predBBox.Vx = predictedFilter.output(2);
                predBBox.Vy = predictedFilter.output(3);
                predBBox.Ax = predictedFilter.output(4);
                predBBox.Ay = predictedFilter.output(5);

                if (!confirmed){
                    this->predictedBBoxesUnconfirmed_.push_back(predBBox);
                }
                else if (missed > 0){
                    this->predictedBBoxesMissed_.push_back(predBBox);
                }
                else{
                    this->predictedBBoxesActive_.push_back(predBBox);
                }
            }
        }
    }
    
    bool dynamicDetector::isCloseToExistingTrack(const joDetector::box3D& currDetectedBBox,
                                                const Eigen::Vector3d& currPcCenter,
                                                const std::vector<bool>& prevMatched)
    {
        (void)currPcCenter;
        (void)prevMatched;

        for (size_t j = 0; j < this->boxHist_.size(); ++j){
            if (this->boxHist_[j].empty()){
                continue;
            }

            joDetector::kalman_filter predictedFilter = this->filters_[j];
            predictedFilter.predict(MatrixXd::Zero(6,1));

            joDetector::box3D predictedBox = this->boxHist_[j][0];
            predictedBox.x = predictedFilter.output(0);
            predictedBox.y = predictedFilter.output(1);
            predictedBox.Vx = predictedFilter.output(2);
            predictedBox.Vy = predictedFilter.output(3);
            predictedBox.Ax = predictedFilter.output(4);
            predictedBox.Ay = predictedFilter.output(5);

            double centerDist = 0.0;
            double iou2d = 0.0;
            double relSizeDiff = 0.0;

            const bool isDuplicate = this->isDetectionDuplicateOfPrediction(currDetectedBBox,
                                                                            predictedBox,
                                                                            centerDist,
                                                                            iou2d,
                                                                            relSizeDiff);

            if (this->enableTrackingDebugLogs_){
                RCLCPP_DEBUG(
                    this->nh_->get_logger(),
                    "[TRACK_NEW_CHECK] candidate_vs_track_id=%d centerDist=%.3f iou2d=%.3f relSizeDiff=%.3f duplicate=%d",
                    static_cast<int>(this->boxHist_[j][0].id),
                    centerDist,
                    iou2d,
                    relSizeDiff,
                    isDuplicate ? 1 : 0);
            }

            if (isDuplicate){
                if (this->enableTrackingDebugLogs_){
                    RCLCPP_DEBUG(
                        this->nh_->get_logger(),
                        "[TRACK_NEW_BLOCKED_BY_EXISTING] candidate_bbox duplicate_of_track_id=%d centerDist=%.3f iou2d=%.3f relSizeDiff=%.3f",
                        static_cast<int>(this->boxHist_[j][0].id),
                        centerDist,
                        iou2d,
                        relSizeDiff);
                }
                return true;
            }
        }

        return false;
    }

    bool dynamicDetector::isDetectionDuplicateOfPrediction(const joDetector::box3D& currDetectedBBox,
                                                        const joDetector::box3D& predictedBox,
                                                        double& centerDist,
                                                        double& iou2d,
                                                        double& relSizeDiff) const
    {
        const double dx = predictedBox.x - currDetectedBBox.x;
        const double dy = predictedBox.y - currDetectedBBox.y;
        centerDist = std::sqrt(dx * dx + dy * dy);

        iou2d = this->computeBoxIoU2D(currDetectedBBox, predictedBox);
        relSizeDiff = this->computeRelativeSizeDiff(currDetectedBBox, predictedBox);

        const bool duplicateByStrongOverlap =
            (centerDist < this->duplicateTrackDistThresh_) &&
            (iou2d > this->duplicateTrackIou2DThresh_) &&
            (relSizeDiff < this->duplicateSizeRelThresh_);

        return duplicateByStrongOverlap;
    }

    double dynamicDetector::computeRelativeSizeDiff(const joDetector::box3D& a,
                                                    const joDetector::box3D& b) const
    {
        const double eps = 1e-6;

        double dx = std::abs(a.x_width - b.x_width) / std::max(std::max(a.x_width, b.x_width), eps);
        double dy = std::abs(a.y_width - b.y_width) / std::max(std::max(a.y_width, b.y_width), eps);
        double dz = std::abs(a.z_width - b.z_width) / std::max(std::max(a.z_width, b.z_width), eps);

        return (dx + dy + dz) / 3.0;
    }

    bool dynamicDetector::areBoxesDuplicate2D(const joDetector::box3D& a,
                                            const joDetector::box3D& b) const
    {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        double iou2d = this->computeBoxIoU2D(a, b);
        double relSizeDiff = this->computeRelativeSizeDiff(a, b);

        if (dist < this->duplicateTrackDistThresh_ &&
            iou2d > this->duplicateTrackIou2DThresh_ &&
            relSizeDiff < this->duplicateSizeRelThresh_)
        {
            return true;
        }

        return false;
    }

    bool dynamicDetector::isDuplicateOfTrackedThisFrame(
        const joDetector::box3D& currDetectedBBox,
        const std::vector<joDetector::box3D>& trackedBBoxesTemp,
        int& duplicateTrackId) const
    {
        duplicateTrackId = -1;

        for (const auto& trackedBox : trackedBBoxesTemp){
            if (this->areBoxesDuplicate2D(currDetectedBBox, trackedBox)){
                duplicateTrackId = trackedBox.id;
                return true;
            }
        }

        return false;
    }

    double dynamicDetector::clampPositive(double value, double minValue) const
    {
        return (value < minValue) ? minValue : value;
    }

    double dynamicDetector::computeAssociationScore(const joDetector::box3D& predictedBox,
                                                    const joDetector::box3D& previousObservedBox,
                                                    const joDetector::box3D& currentBox,
                                                    double dt,
                                                    double& predPosDist,
                                                    double& requiredSpeed,
                                                    double& relSizeDiff,
                                                    double& predIou2d,
                                                    double& prevObsPosDist,
                                                    double& prevObsIou2d,
                                                    std::string& rejectReason) const
    {
        rejectReason.clear();

        const bool lidarOnlyOutsideFov =
            this->isLidarOnlyOutsideFovAssociation(predictedBox, currentBox);

        const double effectiveMaxMatchRange =
            lidarOnlyOutsideFov ? this->maxMatchRangeOutsideFov_ : this->maxMatchRange_;

        const double effectiveMaxMatchSpeed =
            lidarOnlyOutsideFov ? this->maxMatchSpeedOutsideFov_ : this->maxMatchSpeed_;

        const double dxPred = predictedBox.x - currentBox.x;
        const double dyPred = predictedBox.y - currentBox.y;
        predPosDist = std::sqrt(dxPred * dxPred + dyPred * dyPred);

        if (predPosDist >= effectiveMaxMatchRange){
            rejectReason = "REJECT_POS";
            return -1e9;
        }

        dt = this->clampPositive(dt, 1e-3);
        requiredSpeed = predPosDist / dt;

        if (requiredSpeed >= effectiveMaxMatchSpeed){
            rejectReason = "REJECT_SPEED";
            return -1e9;
        }

        relSizeDiff = this->computeRelativeSizeDiff(predictedBox, currentBox);
        predIou2d = this->computeBoxIoU2D(predictedBox, currentBox);

        const double dxPrev = previousObservedBox.x - currentBox.x;
        const double dyPrev = previousObservedBox.y - currentBox.y;
        prevObsPosDist = std::sqrt(dxPrev * dxPrev + dyPrev * dyPrev);
        prevObsIou2d = this->computeBoxIoU2D(previousObservedBox, currentBox);

        const double normPredPosDist =
            predPosDist / this->clampPositive(effectiveMaxMatchRange, 1e-6);

        const double normPrevObsPosDist =
            prevObsPosDist / this->clampPositive(effectiveMaxMatchRange, 1e-6);

        double velDirPenalty = 0.0;
        bool foundTrackIdx = false;
        bool alreadyConfirmed = false;
        int matchedTrackIdx = -1;

        for (size_t j = 0; j < this->boxHist_.size(); ++j){
            if (this->boxHist_[j].empty()){
                continue;
            }

            if (this->boxHist_[j][0].id == predictedBox.id){
                matchedTrackIdx = static_cast<int>(j);
                foundTrackIdx = true;

                if (j < this->confirmedTracks_.size()){
                    alreadyConfirmed = this->confirmedTracks_[j];
                }

                break;
            }
        }

        double effectiveSizeLimit = this->maxRelativeSizeDiffMatch_;

        if (foundTrackIdx && alreadyConfirmed){
            effectiveSizeLimit *= 1.5;
        }

        // Fuori FOV LiDAR-only: DBSCAN può variare parecchio la box.
        // Rilassiamo la size, ma solo in questo caso.
        if (lidarOnlyOutsideFov){
            effectiveSizeLimit = std::max(effectiveSizeLimit,
                                        this->maxRelativeSizeDiffOutsideFov_);
        }

        if (relSizeDiff >= effectiveSizeLimit){
            rejectReason = "REJECT_SIZE";
            return -1e9;
        }

        bool abruptTurnAccepted = false;

        if (foundTrackIdx){
            const bool trackHasYolo =
                matchedTrackIdx >= 0 &&
                matchedTrackIdx < static_cast<int>(this->boxHist_.size()) &&
                !this->boxHist_[matchedTrackIdx].empty() &&
                this->boxHist_[matchedTrackIdx][0].is_yolo_candidate;

            const bool yoloExempt = currentBox.is_yolo_candidate || trackHasYolo;

            bool passVelDir = this->passesVelocityDirectionGate(matchedTrackIdx,
                                                                currentBox,
                                                                alreadyConfirmed,
                                                                yoloExempt);

            if (!passVelDir && alreadyConfirmed && !yoloExempt){
                abruptTurnAccepted =
                    this->isAbruptTurnAllowedOutsideFov(matchedTrackIdx,
                                                        predictedBox,
                                                        currentBox,
                                                        dt);
            }

            if (!passVelDir && !abruptTurnAccepted){
                rejectReason = alreadyConfirmed ?
                            "REJECT_VELDIR_TRACKED" :
                            "REJECT_VELDIR_CONFIRM";
                return -1e9;
            }

            if (!yoloExempt && !alreadyConfirmed &&
                !this->isNaturalMotion(matchedTrackIdx, currentBox))
            {
                rejectReason = "REJECT_NATURAL_MOTION";
                return -1e9;
            }

            velDirPenalty = this->computeVelocityDirectionError(matchedTrackIdx,
                                                                currentBox);
        }

        double yoloClassConsistencyPenalty = 0.0;

        if (foundTrackIdx && predictedBox.is_yolo_candidate){
            if (!currentBox.is_yolo_candidate){
                const Eigen::Vector3d predPos(predictedBox.x,
                                            predictedBox.y,
                                            predictedBox.z);

                if (this->isInCameraFOV(predPos)){
                    yoloClassConsistencyPenalty = 1.0;
                }
            }
        }

        double score =
            - this->matchPosScoreWeight_ * normPredPosDist
            - this->matchSizeScoreWeight_ * relSizeDiff
            + this->matchIou2DScoreWeight_ * predIou2d
            - this->matchPrevObsPosScoreWeight_ * normPrevObsPosDist
            + this->matchPrevObsIou2DScoreWeight_ * prevObsIou2d
            - this->matchVelocityDirectionScoreWeight_ * velDirPenalty
            - this->matchYoloClassConsistencyWeight_ * yoloClassConsistencyPenalty;

        if (foundTrackIdx){
            if (alreadyConfirmed){
                score += this->confirmedTrackAssocBonus_;
            }

            if (currentBox.is_yolo_candidate){
                score += this->yoloTrackAssocBonus_;
            }

            // Bonus solo LiDAR-only fuori FOV.
            // Serve a evitare ID switch / track restart quando la box oscilla leggermente.
            if (lidarOnlyOutsideFov){
                score += alreadyConfirmed ?
                        this->outsideFovConfirmedAssocBonus_ :
                        this->outsideFovTentativeAssocBonus_;
            }

            if (abruptTurnAccepted){
                score += this->outsideFovTurnAssocBonus_;
            }
        }

        const double adaptiveMinScore =
            foundTrackIdx ? this->getAdaptiveMinMatchScore(matchedTrackIdx, currentBox)
                        : this->minMatchScore_;

        if (score < adaptiveMinScore){
            rejectReason = "REJECT_SCORE";
            return score;
        }

        return score;
    }

    bool dynamicDetector::tryAugmentDetectionMatch(int detIdx,
                                                const std::vector<std::vector<int>>& candidatePrevIdxByCurr,
                                                std::vector<int>& assignedPrevToCurr,
                                                std::vector<int>& assignedCurrToPrev,
                                                std::vector<int>& visitTokenByPrev,
                                                int visitToken) const
    {
        for (int prevIdx : candidatePrevIdxByCurr[detIdx]){
            if (prevIdx < 0 || prevIdx >= static_cast<int>(assignedPrevToCurr.size())){
                continue;
            }

            if (visitTokenByPrev[prevIdx] == visitToken){
                continue;
            }
            visitTokenByPrev[prevIdx] = visitToken;

            if (assignedPrevToCurr[prevIdx] == -1){
                assignedPrevToCurr[prevIdx] = detIdx;
                assignedCurrToPrev[detIdx] = prevIdx;
                return true;
            }

            const int otherDetIdx = assignedPrevToCurr[prevIdx];
            if (otherDetIdx >= 0 &&
                this->tryAugmentDetectionMatch(otherDetIdx,
                                            candidatePrevIdxByCurr,
                                            assignedPrevToCurr,
                                            assignedCurrToPrev,
                                            visitTokenByPrev,
                                            visitToken))
            {
                assignedPrevToCurr[prevIdx] = detIdx;
                assignedCurrToPrev[detIdx] = prevIdx;
                return true;
            }
        }

        return false;
    }

    void dynamicDetector::assignMatchesGlobally(const std::vector<joDetector::matchCandidate>& candidates,
                                                int numCurr,
                                                int numPrev,
                                                std::vector<int>& bestMatch) const
    {
        bestMatch.assign(numCurr, -1);

        std::vector<std::vector<int>> candidatePrevIdxByCurr(numCurr);

        // I candidati arrivano già ordinati dal migliore al peggiore.
        for (const auto& candidate : candidates){
            if (candidate.currIdx < 0 || candidate.currIdx >= numCurr){
                continue;
            }
            if (candidate.prevIdx < 0 || candidate.prevIdx >= numPrev){
                continue;
            }
            candidatePrevIdxByCurr[candidate.currIdx].push_back(candidate.prevIdx);
        }

        std::vector<int> assignedPrevToCurr(numPrev, -1);
        std::vector<int> assignedCurrToPrev(numCurr, -1);
        std::vector<int> visitTokenByPrev(numPrev, -1);

        int visitToken = 0;
        for (int detIdx = 0; detIdx < numCurr; ++detIdx){
            ++visitToken;
            this->tryAugmentDetectionMatch(detIdx,
                                        candidatePrevIdxByCurr,
                                        assignedPrevToCurr,
                                        assignedCurrToPrev,
                                        visitTokenByPrev,
                                        visitToken);
        }

        for (int detIdx = 0; detIdx < numCurr; ++detIdx){
            bestMatch[detIdx] = assignedCurrToPrev[detIdx];
        }
    }

    void dynamicDetector::getDynamicPc(std::vector<Eigen::Vector3d>& dynamicPc){
        Eigen::Vector3d curPoint;
        for (size_t i=0; i<this->filteredPcClusters_.size(); ++i){
            for (size_t j=0; j<this->filteredPcClusters_[i].size(); ++j){
                curPoint = this->filteredPcClusters_[i][j];
                for (size_t k=0; k<this->dynamicBBoxes_.size(); ++k){
                    if (abs(curPoint(0)-this->dynamicBBoxes_[k].x)<=this->dynamicBBoxes_[k].x_width/2 and 
                        abs(curPoint(1)-this->dynamicBBoxes_[k].y)<=this->dynamicBBoxes_[k].y_width/2 and 
                        abs(curPoint(2)-this->dynamicBBoxes_[k].z)<=this->dynamicBBoxes_[k].z_width/2) {
                        dynamicPc.push_back(curPoint);
                        break;
                    }
                }
            }
        }
    } 
    
    void dynamicDetector::publishUVImages(){
        if (this->uvDetector_ != NULL){
            std_msgs::msg::Header depthHeader;
            depthHeader.stamp = this->nh_->now();
            depthHeader.frame_id = this->tfDepthFrame_;

            auto depthBoxMsg = cv_bridge::CvImage(depthHeader, "bgr8", this->uvDetector_->depth_show).toImageMsg();
            auto UmapBoxMsg = cv_bridge::CvImage(depthHeader, "bgr8", this->uvDetector_->U_map_show).toImageMsg();
            auto birdBoxMsg = cv_bridge::CvImage(depthHeader, "bgr8", this->uvDetector_->bird_view).toImageMsg();  
            this->uvDepthMapPub_->publish(*depthBoxMsg);
            this->uDepthMapPub_->publish(*UmapBoxMsg); 
            this->uvBirdViewPub_->publish(*birdBoxMsg);
        }     
    }


    void dynamicDetector::publishColorImages(){
        std_msgs::msg::Header colorHeader;
        colorHeader.stamp = this->nh_->now();
        colorHeader.frame_id = this->tfColorFrame_;
        auto detectedColorImgMsg = cv_bridge::CvImage(colorHeader, "rgb8", this->detectedColorImage_).toImageMsg();
        this->detectedColorImgPub_->publish(*detectedColorImgMsg);
    }

    void dynamicDetector::publishPoints(const std::vector<Eigen::Vector3d>& points, const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher){
        pcl::PointXYZ pt;
        pcl::PointCloud<pcl::PointXYZ> cloud;        
        for (size_t i=0; i<points.size(); ++i){
            pt.x = points[i](0);
            pt.y = points[i](1);
            pt.z = points[i](2);
            cloud.push_back(pt);
        }    
        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = "odom";

        sensor_msgs::msg::PointCloud2 cloudMsg;
        pcl::toROSMsg(cloud, cloudMsg);
        publisher->publish(cloudMsg);
    }


    void dynamicDetector::publish3dBox(const std::vector<box3D>& boxes,
                                const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher,
                                double r, double g, double b, double a, bool highlightYolo, bool useClassificationColors, double lifetimeSeconds){
        visualization_msgs::msg::MarkerArray markers;

        for (size_t i = 0; i < boxes.size(); i++)
        {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = "odom";
            line.ns = "box3D";
            line.id = i;
            line.type = visualization_msgs::msg::Marker::LINE_LIST;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.scale.x = 0.06;
            if (highlightYolo && boxes[i].is_yolo_candidate){
                line.color.r = 0.45;
                line.color.g = 0.35;
                line.color.b = 0.75;
            } else if (useClassificationColors && boxes[i].is_dynamic){
                line.color.r = 0.0;
                line.color.g = 0.0;
                line.color.b = 1.0;
            } else if (useClassificationColors && boxes[i].is_potentially_dynamic){
                line.color.r = 0.0;
                line.color.g = 1.0;
                line.color.b = 0.0;
            } else {
                line.color.r = r;
                line.color.g = g;
                line.color.b = b;
            }
            line.color.a = a;
            line.lifetime = rclcpp::Duration::from_seconds(lifetimeSeconds > 0.0 ? lifetimeSeconds : this->dt_);
            line.pose.orientation.x = 0.0;
            line.pose.orientation.y = 0.0;
            line.pose.orientation.z = 0.0;
            line.pose.orientation.w = 1.0;

            line.pose.position.x = boxes[i].x;
            line.pose.position.y = boxes[i].y;
            line.pose.position.z = boxes[i].z;

            double half_x = boxes[i].x_width / 2.0;
            double half_y = boxes[i].y_width / 2.0;
            double half_z = boxes[i].z_width / 2.0;

            geometry_msgs::msg::Point corner[8];
            corner[0].x = -half_x; corner[0].y = -half_y; corner[0].z = -half_z;
            corner[1].x = -half_x; corner[1].y =  half_y; corner[1].z = -half_z;
            corner[2].x =  half_x; corner[2].y =  half_y; corner[2].z = -half_z;
            corner[3].x =  half_x; corner[3].y = -half_y; corner[3].z = -half_z;
            corner[4].x = -half_x; corner[4].y = -half_y; corner[4].z =  half_z;
            corner[5].x = -half_x; corner[5].y =  half_y; corner[5].z =  half_z;
            corner[6].x =  half_x; corner[6].y =  half_y; corner[6].z =  half_z;
            corner[7].x =  half_x; corner[7].y = -half_y; corner[7].z =  half_z;

            int edgeIdx[12][2] = {
                {0,1}, {1,2}, {2,3}, {3,0},
                {4,5}, {5,6}, {6,7}, {7,4},
                {0,4}, {1,5}, {2,6}, {3,7}
            };

            for (int e = 0; e < 12; e++)
            {
                line.points.push_back(corner[edgeIdx[e][0]]);
                line.points.push_back(corner[edgeIdx[e][1]]);
            }

            markers.markers.push_back(line);
        }

        publisher->publish(markers);
    }


    void dynamicDetector::publishHistoryTraj(){
        // Build a set of track IDs currently in the output (trackedBBoxes_).
        // Only draw trajectories for active tracks — not for lost/unmatched tracks
        // that are still alive internally but have no current detection.
        std::unordered_set<int> activeIds;
        for (const auto& tb : this->trackedBBoxes_){
            activeIds.insert(static_cast<int>(tb.id));
        }

        visualization_msgs::msg::MarkerArray trajMsg;
        int countMarker = 0;
        for (size_t i=0; i<this->boxHist_.size(); ++i){
            if (this->boxHist_[i].size() <= 1) continue;

            // Skip tracks not currently in the output
            const int trackId = static_cast<int>(this->boxHist_[i][0].id);
            if (activeIds.find(trackId) == activeIds.end()) continue;

            visualization_msgs::msg::Marker traj;
            traj.header.frame_id = "odom";
            traj.header.stamp = this->nh_->now();
            traj.ns = "dynamic_detector";
            traj.id = countMarker;
            traj.type = visualization_msgs::msg::Marker::LINE_LIST;
            traj.scale.x = 0.03;
            traj.scale.y = 0.03;
            traj.scale.z = 0.03;
            traj.color.a = 1.0;
            traj.color.r = 0.0;
            traj.color.g = 1.0;
            traj.color.b = 0.0;
            traj.pose.orientation.w = 1.0;
            traj.pose.orientation.x = 0.0;
            traj.pose.orientation.y = 0.0;
            traj.pose.orientation.z = 0.0;
            for (size_t j=0; j<this->boxHist_[i].size()-1; ++j){
                geometry_msgs::msg::Point p1, p2;
                joDetector::box3D box1 = this->boxHist_[i][j];
                joDetector::box3D box2 = this->boxHist_[i][j+1];
                p1.x = box1.x; p1.y = box1.y; p1.z = box1.z;
                p2.x = box2.x; p2.y = box2.y; p2.z = box2.z;
                traj.points.push_back(p1);
                traj.points.push_back(p2);
            }

            ++countMarker;
            trajMsg.markers.push_back(traj);
        }
        this->historyTrajPub_->publish(trajMsg);
    }

    void dynamicDetector::publishVelVis(){ // publish velocities for all tracked objects
        visualization_msgs::msg::MarkerArray velVisMsg;
        int countMarker = 0;
        for (size_t i=0; i<this->trackedBBoxes_.size(); ++i){
            visualization_msgs::msg::Marker velMarker;
            velMarker.header.frame_id = "odom";
            velMarker.header.stamp = this->nh_->now();
            velMarker.ns = "dynamic_detector";
            velMarker.id =  countMarker;
            velMarker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            velMarker.pose.position.x = this->trackedBBoxes_[i].x;
            velMarker.pose.position.y = this->trackedBBoxes_[i].y;
            velMarker.pose.position.z = this->trackedBBoxes_[i].z + this->trackedBBoxes_[i].z_width/2. + 0.3;
            velMarker.scale.x = 0.15;
            velMarker.scale.y = 0.15;
            velMarker.scale.z = 0.15;
            velMarker.color.a = 1.0;
            velMarker.color.r = 1.0;
            velMarker.color.g = 0.0;
            velMarker.color.b = 0.0;
            velMarker.lifetime = rclcpp::Duration::from_seconds(0.1);
            double vx = this->trackedBBoxes_[i].Vx;
            double vy = this->trackedBBoxes_[i].Vy;
            double vNorm = sqrt(vx*vx+vy*vy);
            std::string velText = "Vx=" + std::to_string(vx) + ", Vy=" + std::to_string(vy) + ", |V|=" + std::to_string(vNorm);
            velMarker.text = velText;
            velVisMsg.markers.push_back(velMarker);
            ++countMarker;
        }
        this->velVisPub_->publish(velVisMsg);
    }

    void dynamicDetector::publishLidarClusters(){
        sensor_msgs::msg::PointCloud2 lidarClustersMsg;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (size_t i=0; i<this->lidarClusters_.size(); ++i){
            joDetector::Cluster & cluster = this->lidarClusters_[i];

            std_msgs::msg::ColorRGBA color;
            srand(cluster.cluster_id);
            color.r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            color.g = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            color.b = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            // color.r = 0.5;
            // color.g = 0.5;
            // color.b = 0.5;
            // color.a = 1.0;

            for (size_t j=0; j<cluster.points->size(); ++j){
                pcl::PointXYZRGB point;
                const pcl::PointXYZ & pt = cluster.points->at(j);
                point.x = pt.x;
                point.y = pt.y;
                point.z = pt.z;
                point.r = color.r * 255;
                point.g = color.g * 255;
                point.b = color.b * 255;
                colored_cloud->push_back(point);
            }
        }
        pcl::toROSMsg(*colored_cloud, lidarClustersMsg);
        lidarClustersMsg.header.frame_id = "odom";
        lidarClustersMsg.header.stamp = this->nh_->now();
        this->lidarClustersPub_->publish(lidarClustersMsg);
    }

    void dynamicDetector::publishFilteredPoints(){
        sensor_msgs::msg::PointCloud2 filteredPointsMsg;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (size_t i=0; i<this->filteredPcClusters_.size(); ++i){
            std_msgs::msg::ColorRGBA color;
            color.r = 0.5;
            color.g = 0.5;
            color.b = 0.5;
            color.a = 1.0;

            for (size_t j=0; j<this->filteredPcClusters_[i].size(); ++j){
                pcl::PointXYZRGB point;
                point.x = this->filteredPcClusters_[i][j](0);
                point.y = this->filteredPcClusters_[i][j](1);
                point.z = this->filteredPcClusters_[i][j](2);
                point.r = color.r * 255;
                point.g = color.g * 255;
                point.b = color.b * 255;
                colored_cloud->push_back(point);
            }
        }
        pcl::toROSMsg(*colored_cloud, filteredPointsMsg);
        filteredPointsMsg.header.frame_id = "odom";
        filteredPointsMsg.header.stamp = this->nh_->now();
        this->filteredPointsPub_->publish(filteredPointsMsg);
    }

    void dynamicDetector::publishRawDynamicPoints(){
        if (not this->latestCloud_){
            return;
        }
        try {
            pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (this->hasSensorPose_) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::fromROSMsg(*this->latestCloud_, *tempCloud);
                
                Eigen::Affine3d transform = Eigen::Affine3d::Identity();
                transform.linear() = this->orientationLidar_;
                transform.translation() = this->positionLidar_;
                
                pcl::transformPointCloud(*tempCloud, *globalCloud, transform);
                sensor_msgs::msg::PointCloud2 cloudMsg;
                pcl::toROSMsg(*globalCloud, cloudMsg);
                cloudMsg.header.frame_id = "odom";
                cloudMsg.header.stamp = this->nh_->now();
                this->rawLidarPointsPub_->publish(cloudMsg);
            }
            else {
                pcl::fromROSMsg(*this->latestCloud_, *globalCloud);
            }
            
            std::vector<Eigen::Vector3d> dynamicEigenPoints;
            
            for (const auto& box : this->dynamicBBoxes_) {
                if (!box.is_dynamic)
                    continue;
                
                double xmin = box.x - box.x_width / 2.0;
                double xmax = box.x + box.x_width / 2.0;
                double ymin = box.y - box.y_width / 2.0;
                double ymax = box.y + box.y_width / 2.0;
                double zmin = box.z - box.z_width / 2.0;
                double zmax = box.z + box.z_width / 2.0;
                
                for (const auto& point : globalCloud->points) {
                    if (point.x >= xmin && point.x <= xmax &&
                        point.y >= ymin && point.y <= ymax &&
                        point.z >= zmin && point.z <= zmax)
                    {
                        dynamicEigenPoints.push_back(Eigen::Vector3d(point.x, point.y, point.z));
                    }
                }
            }
            
            if (dynamicEigenPoints.empty()) {
                return;
            }
            
            this->publishPoints(dynamicEigenPoints, this->rawDynamicPointsPub_);
        }
        catch (const pcl::PCLException& e) {
            RCLCPP_ERROR(this->nh_->get_logger(), "PCL Exception during dynamic point extraction: %s", e.what());
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->nh_->get_logger(), "Standard Exception during dynamic point extraction: %s", e.what());
        }
        catch (...) {
            RCLCPP_ERROR(this->nh_->get_logger(), "Unknown error during dynamic point extraction.");
        }
    }

    void dynamicDetector::transformBBox(const Eigen::Vector3d& center, const Eigen::Vector3d& size, const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation,
                                               Eigen::Vector3d& newCenter, Eigen::Vector3d& newSize){
        double x = center(0); 
        double y = center(1);
        double z = center(2);
        double xWidth = size(0);
        double yWidth = size(1);
        double zWidth = size(2);

        // get 8 bouding boxes coordinates in the camera frame
        Eigen::Vector3d p1 (x+xWidth/2.0, y+yWidth/2.0, z+zWidth/2.0);
        Eigen::Vector3d p2 (x+xWidth/2.0, y+yWidth/2.0, z-zWidth/2.0);
        Eigen::Vector3d p3 (x+xWidth/2.0, y-yWidth/2.0, z+zWidth/2.0);
        Eigen::Vector3d p4 (x+xWidth/2.0, y-yWidth/2.0, z-zWidth/2.0);
        Eigen::Vector3d p5 (x-xWidth/2.0, y+yWidth/2.0, z+zWidth/2.0);
        Eigen::Vector3d p6 (x-xWidth/2.0, y+yWidth/2.0, z-zWidth/2.0);
        Eigen::Vector3d p7 (x-xWidth/2.0, y-yWidth/2.0, z+zWidth/2.0);
        Eigen::Vector3d p8 (x-xWidth/2.0, y-yWidth/2.0, z-zWidth/2.0);

        // transform 8 points to the map coordinate frame
        Eigen::Vector3d p1m = orientation * p1 + position;
        Eigen::Vector3d p2m = orientation * p2 + position;
        Eigen::Vector3d p3m = orientation * p3 + position;
        Eigen::Vector3d p4m = orientation * p4 + position;
        Eigen::Vector3d p5m = orientation * p5 + position;
        Eigen::Vector3d p6m = orientation * p6 + position;
        Eigen::Vector3d p7m = orientation * p7 + position;
        Eigen::Vector3d p8m = orientation * p8 + position;
        std::vector<Eigen::Vector3d> pointsMap {p1m, p2m, p3m, p4m, p5m, p6m, p7m, p8m};

        // find max min in x, y, z directions
        double xmin=p1m(0); double xmax=p1m(0); 
        double ymin=p1m(1); double ymax=p1m(1);
        double zmin=p1m(2); double zmax=p1m(2);
        for (Eigen::Vector3d pm : pointsMap){
            if (pm(0) < xmin){xmin = pm(0);}
            if (pm(0) > xmax){xmax = pm(0);}
            if (pm(1) < ymin){ymin = pm(1);}
            if (pm(1) > ymax){ymax = pm(1);}
            if (pm(2) < zmin){zmin = pm(2);}
            if (pm(2) > zmax){zmax = pm(2);}
        }
        newCenter(0) = (xmin + xmax)/2.0;
        newCenter(1) = (ymin + ymax)/2.0;
        newCenter(2) = (zmin + zmax)/2.0;
        newSize(0) = xmax - xmin;
        newSize(1) = ymax - ymin;
        newSize(2) = zmax - zmin;
    }

    // user functions
    void dynamicDetector::getDynamicObstacles(std::vector<joDetector::box3D>& incomeDynamicBBoxes, const Eigen::Vector3d &robotSize){
        incomeDynamicBBoxes.clear();
        for (int i=0; i<int(this->dynamicBBoxes_.size()); i++){
            joDetector::box3D box = this->dynamicBBoxes_[i];
            box.x_width += robotSize(0);
            box.y_width += robotSize(1);
            box.z_width += robotSize(2);
            incomeDynamicBBoxes.push_back(box);
        }
    }
}
