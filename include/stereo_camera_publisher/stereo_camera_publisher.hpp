#ifndef STEREO_CAMERA_PUBLISHER_HPP_
#define STEREO_CAMERA_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include <opencv2/opencv.hpp>

class StereoCameraPublisher : public rclcpp::Node
{
public:
    StereoCameraPublisher();
    ~StereoCameraPublisher();

    bool initialize();
    void run();

private:
    void captureAndPublish();

    // ROS2 publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Camera info managers
    std::shared_ptr<camera_info_manager::CameraInfoManager> left_info_manager_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> right_info_manager_;

    // OpenCV video capture
    cv::VideoCapture camera_;

    // Parameters
    std::string camera_device_;
    int image_width_;
    int image_height_;
    double frame_rate_;
    std::string left_topic_;
    std::string right_topic_;
    std::string left_frame_id_;
    std::string right_frame_id_;
    int exposure_;
    int gain_;
    int brightness_;
    int backlight_compensation_;
    int sharpness_;
    int white_balance_automatic_;
    int contrast_;
    std::string left_camera_info_url_;
    std::string right_camera_info_url_;

    // State flags
    bool cameras_initialized_;
};

#endif  // STEREO_CAMERA_PUBLISHER_HPP_
