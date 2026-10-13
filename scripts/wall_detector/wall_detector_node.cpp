/*
    FILE: wall_detector_node.cpp
    --------------------------
    Run wall detector node
*/
#include <rclcpp/rclcpp.hpp>
#include "wallDetector.h"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto nh = std::make_shared<rclcpp::Node>("wall_detector_node", options);

    joDetector::WallDetector wd(nh);

    rclcpp::spin(nh);
    rclcpp::shutdown();

    return 0;
}
