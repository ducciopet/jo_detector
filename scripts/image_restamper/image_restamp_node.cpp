/*
    FILE: image_restamp_node.cpp
    -----------------------------
    Generic image_transport decode + restamp bridge.

    Root cause this works around: at least the zstd_image_transport
    subscriber plugin (compressed_image_transport for JPEG may or may not be
    affected too, see below) does not propagate the original message's
    header.stamp through decode — the decoded sensor_msgs::msg::Image always
    comes out stamped (0, 0). calibration_icp_node.cpp hit this first (see
    its own comments) and works around it internally with a manual
    Cache-based sync using this->now() instead of the broken stamp. That
    fix is local to that one node; anything else that subscribes the same
    compressed/zstd topic directly (e.g. rtabmap_odom's rgbd_odometry, whose
    message_filters::Synchronizer<ApproximateTime<...>> compares the
    *decoded* stamps) never finds a match — depth sits at stamp=0 forever
    while color/camera_info carry real timestamps, so approx_sync never
    fires, regardless of the configured tolerance.

    This node is the reusable version of that same workaround: subscribe
    once via image_transport with whatever transport plugin the topic
    actually needs (zstd, compressed, compressedDepth, raw, ...), and
    republish a plain sensor_msgs::msg::Image with header.stamp overwritten
    to this->now() (which — under use_sim_time — tracks /clock, i.e. the
    bag's own timeline, not wall-clock, so it stays close to the other
    topics' real recorded stamps rather than drifting). Point any consumer
    that needs correctly-stamped raw images (e.g. rgbd_odometry with
    image_transport/depth_transport left at the default "raw") at the
    output_topic instead of subscribing the compressed/zstd stream itself.

    Parameters:
      input_topic      (string, required) base topic, e.g.
                        /front_camera/camera/aligned_depth_to_color/image_raw
      input_transport   (string, default "raw") image_transport plugin the
                        input topic actually needs: "zstd", "compressed",
                        "compressedDepth", "raw", ...
      output_topic      (string, required) plain sensor_msgs/Image topic to
                        republish on (raw transport, no suffix).
*/
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImageRestampNode : public rclcpp::Node {
public:
    ImageRestampNode() : rclcpp::Node("image_restamp_node") {
        input_topic_ = this->declare_parameter<std::string>("input_topic", "");
        input_transport_ = this->declare_parameter<std::string>("input_transport", "raw");
        output_topic_ = this->declare_parameter<std::string>("output_topic", "");

        if (input_topic_.empty() || output_topic_.empty()) {
            RCLCPP_FATAL(this->get_logger(),
                "image_restamp_node requires both 'input_topic' and 'output_topic' parameters.");
            throw std::runtime_error("image_restamp_node: missing required parameters");
        }

        // Plain reliable QoS (rclcpp default), not SensorDataQoS: rgbd_odometry's
        // own image_transport::SubscriberFilter for the "raw" transport requests
        // RELIABLE, and a BEST_EFFORT publisher here is QoS-incompatible with
        // that — DDS matches the endpoints but silently delivers nothing (no
        // error beyond a one-time warning), which looks identical to the
        // original stamp-mismatch sync failure this node exists to fix.
        pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, rclcpp::QoS(10));

        // image_transport::create_subscription needs a raw rclcpp::Node*, not
        // a shared_from_this() (unavailable this early in construction).
        sub_ = image_transport::create_subscription(
            this, input_topic_,
            std::bind(&ImageRestampNode::onImage, this, std::placeholders::_1),
            input_transport_, rmw_qos_profile_sensor_data);

        RCLCPP_INFO(this->get_logger(),
            "Restamping '%s' (transport=%s) -> '%s'",
            input_topic_.c_str(), input_transport_.c_str(), output_topic_.c_str());
    }

private:
    void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        sensor_msgs::msg::Image out = *msg;
        out.header.stamp = this->now();
        pub_->publish(out);
    }

    std::string input_topic_;
    std::string input_transport_;
    std::string output_topic_;
    image_transport::Subscriber sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageRestampNode>());
    rclcpp::shutdown();
    return 0;
}
