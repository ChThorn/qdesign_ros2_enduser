// launch/include/camera_node.hpp
#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <librealsense2/rs.hpp>

namespace realsense_launch {

class CameraNode : public rclcpp::Node {
public:
    explicit CameraNode(const rclcpp::NodeOptions & options)
    : Node("realsense_node", options) {
        width_ = this->declare_parameter("width", 640);
        height_ = this->declare_parameter("height", 480);
        fps_ = this->declare_parameter("fps", 30);
        
        pipe = std::make_unique<rs2::pipeline>();
        cfg = std::make_unique<rs2::config>();
        
        cfg->enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, fps_);
        cfg->enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, fps_);
        
        try {
            pipe->start(*cfg);
            RCLCPP_INFO(this->get_logger(), "RealSense pipeline started successfully");
        }
        catch(const rs2::error & e) {
            RCLCPP_ERROR(this->get_logger(), "RealSense error calling start(): %s", e.what());
            return;
        }
        
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "camera/color/image_raw", 10);
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "camera/depth/image_rect_raw", 10);
            
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000/fps_),
            std::bind(&CameraNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Camera node initialized");
    }

    ~CameraNode() {
        if (pipe) {
            pipe->stop();
        }
    }

private:
    void timer_callback() {
        try {
            rs2::frameset frames = pipe->wait_for_frames();
            auto color_frame = frames.get_color_frame();
            auto depth_frame = frames.get_depth_frame();
            
            if (color_frame && depth_frame) {
                publish_frame(color_frame, image_pub_, "bgr8");
                publish_frame(depth_frame, depth_pub_, "16UC1");
            }
        }
        catch(const rs2::error & e) {
            RCLCPP_ERROR(this->get_logger(), "RealSense error in callback: %s", e.what());
        }
    }
    
    void publish_frame(const rs2::video_frame& frame,
                      const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& publisher,
                      const std::string& encoding) {
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        
        msg->header.frame_id = "camera_link";
        msg->header.stamp = this->now();
        msg->height = frame.get_height();
        msg->width = frame.get_width();
        msg->encoding = encoding;
        msg->is_bigendian = false;
        msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.get_stride_in_bytes());
        
        size_t data_size = msg->step * msg->height;
        msg->data.resize(data_size);
        memcpy(msg->data.data(), frame.get_data(), data_size);
        
        publisher->publish(*msg);
    }

    int width_;
    int height_;
    int fps_;
    std::unique_ptr<rs2::pipeline> pipe;
    std::unique_ptr<rs2::config> cfg;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace realsense_launch

#endif // CAMERA_NODE_HPP