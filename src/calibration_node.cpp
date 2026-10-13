#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>

class CalibrationNode : public rclcpp::Node {
public:
    CalibrationNode() : Node("calibration_node") {
        // Initialize TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

        // Get topic names from parameters (with defaults)
        std::string depth_topic = this->declare_parameter("depth_topic", std::string("/camera1/rs1/depth/image_rect_raw"));
        std::string velodyne_topic = this->declare_parameter("velodyne_topic", std::string("/velodyne_points"));
        
        // Load camera intrinsics from config
        std::vector<double> depth_intrinsics_vec = this->declare_parameter(
            "jo_detector.depth_intrinsics", 
            std::vector<double>{645.0, 644.3, 653.9, 365.6});
        
        double depth_scale = this->declare_parameter(
            "jo_detector.depth_scale_factor", 1000.0);
        
        double depth_min = this->declare_parameter(
            "jo_detector.depth_min_value", 0.5);
        
        double depth_max = this->declare_parameter(
            "jo_detector.depth_max_value", 5.0);
        
        int depth_skip = this->declare_parameter(
            "jo_detector.depth_skip_pixel", 2);
        
        // Store camera intrinsics
        fx_ = depth_intrinsics_vec[0];
        fy_ = depth_intrinsics_vec[1];
        cx_ = depth_intrinsics_vec[2];
        cy_ = depth_intrinsics_vec[3];
        depth_scale_ = depth_scale;
        depth_min_ = depth_min;
        depth_max_ = depth_max;
        depth_skip_ = depth_skip;
        
        RCLCPP_INFO(this->get_logger(), "Calibration node starting");
        RCLCPP_INFO(this->get_logger(), "  Velodyne topic: %s", velodyne_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Depth topic: %s", depth_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Camera intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                   fx_, fy_, cx_, cy_);
        RCLCPP_INFO(this->get_logger(), "  Depth scale: %.1f, min: %.2f, max: %.2f, skip: %d", 
                   depth_scale_, depth_min_, depth_max_, depth_skip_);

        // Publishers
        pub_velodyne_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/calibration/velodyne_points", 10);
        pub_depth_transformed_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/calibration/depth_cloud_in_velodyne", 10);

        // Subscribers with synchronization (only velodyne and depth image).
        // "compressedDepth" transport: compressed_depth_image_transport
        // decompresses transparently on receipt, so onData() below still
        // gets a normal, uncompressed sensor_msgs::msg::Image.
        sub_velodyne_.subscribe(this, velodyne_topic);
        sub_depth_.subscribe(this, depth_topic, "compressedDepth", rmw_qos_profile_sensor_data);

        // Synchronizer (ApproximateTime with larger queue for bag playback)
        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::PointCloud2,
            sensor_msgs::msg::Image> SyncPolicy;

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(100), sub_velodyne_, sub_depth_);
        sync_->registerCallback(&CalibrationNode::onData, this);

        RCLCPP_INFO(this->get_logger(), "Calibration node initialized - waiting for synchronized messages...");
    }

private:
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_velodyne_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_depth_transformed_;

    // Subscribers (only velodyne and depth)
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_velodyne_;
    image_transport::SubscriberFilter sub_depth_;

    // Synchronizer
    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::PointCloud2,
            sensor_msgs::msg::Image>>> sync_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Camera intrinsics
    double fx_, fy_, cx_, cy_;
    double depth_scale_, depth_min_, depth_max_;
    int depth_skip_;
    
    int sync_count_ = 0;

    void onData(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& velodyne_msg,
                const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
        
        sync_count_++;
        if (sync_count_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), "Received synchronized messages (%d total)", sync_count_);
            RCLCPP_INFO(this->get_logger(), "  Velodyne: %u points, frame: %s", 
                       velodyne_msg->width * velodyne_msg->height, velodyne_msg->header.frame_id.c_str());
            RCLCPP_INFO(this->get_logger(), "  Depth: %ux%u, frame: %s", 
                       depth_msg->width, depth_msg->height, depth_msg->header.frame_id.c_str());
        }
        
        // Publish velodyne points as-is (already in velodyne frame)
        auto velodyne_out = std::make_shared<sensor_msgs::msg::PointCloud2>(*velodyne_msg);
        velodyne_out->header.frame_id = "velodyne";
        pub_velodyne_->publish(*velodyne_out);

        // Convert depth image to point cloud
        try {
            auto depth_cv = cv_bridge::toCvShare(depth_msg);
            auto depth_cloud = depthToPointCloud(depth_cv->image);

            if (depth_cloud->size() == 0) {
                RCLCPP_WARN(this->get_logger(), "Depth cloud is empty!");
                return;
            }

            // Set frame to camera frame (use rs1_link which exists in TF tree)
            depth_cloud->header.frame_id = "rs1_link";
            // Convert ROS Time to PCL timestamp (nanoseconds since epoch)
            uint64_t timestamp_ns = depth_msg->header.stamp.sec * 1000000000UL + 
                                    depth_msg->header.stamp.nanosec;
            depth_cloud->header.stamp = timestamp_ns;

            // Transform to velodyne frame
            auto depth_in_velodyne = transformPointCloud(depth_cloud, "velodyne");
            
            if (depth_in_velodyne) {
                depth_in_velodyne->header.frame_id = "velodyne";
                pub_depth_transformed_->publish(*depth_in_velodyne);
            }

        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error processing depth: %s", e.what());
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr depthToPointCloud(const cv::Mat& depth_image) {
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        
        for (int y = 0; y < depth_image.rows; y += depth_skip_) {
            for (int x = 0; x < depth_image.cols; x += depth_skip_) {
                uint16_t d = depth_image.at<uint16_t>(y, x);
                
                if (d == 0) continue; // Invalid depth
                
                float Z = d / depth_scale_; // Convert to meters
                
                if (Z < depth_min_ || Z > depth_max_) continue; // Filter by depth range
                
                float X = (x - cx_) * Z / fx_;
                float Y = (y - cy_) * Z / fy_;
                
                cloud->push_back(pcl::PointXYZ(X, Y, Z));
            }
        }

        return cloud;
    }

    sensor_msgs::msg::PointCloud2::SharedPtr transformPointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::string& target_frame) {

        try {
            // Get transform from camera frame to target frame
            std::string source_frame = cloud->header.frame_id;
            
            // Convert PCL timestamp to ROS Time
            uint64_t timestamp_ns = cloud->header.stamp;
            rclcpp::Time stamp(timestamp_ns);
            
            RCLCPP_DEBUG(this->get_logger(), "Looking up transform from %s to %s", 
                        source_frame.c_str(), target_frame.c_str());
            
            auto transform = tf_buffer_->lookupTransform(
                target_frame, source_frame,
                stamp,
                std::chrono::milliseconds(100));

            // Extract rotation and translation
            tf2::Transform tf_transform;
            tf2::fromMsg(transform.transform, tf_transform);

            // Transform points
            auto transformed_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            for (const auto& pt : cloud->points) {
                tf2::Vector3 point(pt.x, pt.y, pt.z);
                tf2::Vector3 transformed_point = tf_transform * point;
                transformed_cloud->push_back(pcl::PointXYZ(
                    transformed_point.x(),
                    transformed_point.y(),
                    transformed_point.z()));
            }

            // Convert to ROS message
            auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            pcl::toROSMsg(*transformed_cloud, *cloud_msg);
            
            // Convert PCL timestamp back to ROS message format
            uint64_t timestamp_ns_out = cloud->header.stamp;
            cloud_msg->header.stamp.sec = timestamp_ns_out / 1000000000UL;
            cloud_msg->header.stamp.nanosec = timestamp_ns_out % 1000000000UL;
            cloud_msg->header.frame_id = target_frame;

            return cloud_msg;

        } catch (const tf2::TransformException& e) {
            RCLCPP_WARN(this->get_logger(), "Transform lookup failed from %s to %s: %s", 
                       cloud->header.frame_id.c_str(), target_frame.c_str(), e.what());
            return nullptr;
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CalibrationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
