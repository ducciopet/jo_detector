/*
	FILE: detector_node.cpp
	--------------------------
	Run detector node
*/
#include <rclcpp/rclcpp.hpp>
#include <jo_detector/dynamicDetector.h>

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
	options.automatically_declare_parameters_from_overrides(true);
	auto nh = std::make_shared<rclcpp::Node>("detector_node", options);

	joDetector::dynamicDetector d (nh);

	rclcpp::spin(nh);
	rclcpp::shutdown();

	return 0;
}