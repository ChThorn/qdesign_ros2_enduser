#include <rclcpp/rclcpp.hpp>
#include "include/camera_node.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<realsense_launch::CameraNode>(rclcpp::NodeOptions());
    executor.add_node(node);
    
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}