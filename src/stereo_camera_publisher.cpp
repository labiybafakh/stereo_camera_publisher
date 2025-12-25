#include "stereo_camera_publisher/stereo_camera_publisher.hpp"
#include <cv_bridge/cv_bridge.h>

StereoCameraPublisher::StereoCameraPublisher()
    : Node("stereo_camera_publisher"),
      cameras_initialized_(false)
{
    // Declare parameters
    this->declare_parameter<std::string>("camera_device", "/dev/video2");
    this->declare_parameter<int>("image_width", 1280);
    this->declare_parameter<int>("image_height", 720);
    this->declare_parameter<double>("frame_rate", 30.0);
    this->declare_parameter<std::string>("left_topic", "/stereo/left/image_raw");
    this->declare_parameter<std::string>("right_topic", "/stereo/right/image_raw");
    this->declare_parameter<std::string>("left_frame_id", "stereo_left");
    this->declare_parameter<std::string>("right_frame_id", "stereo_right");
    this->declare_parameter<int>("exposure", 3000);
    this->declare_parameter<int>("gain", 120);
    this->declare_parameter<int>("brightness", 20);
    this->declare_parameter<int>("backlight_compensation", 100);
    this->declare_parameter<int>("sharpness", 12);
    this->declare_parameter<int>("white_balance_automatic", 0);
    this->declare_parameter<int>("contrast", 38);
    this->declare_parameter<std::string>("left_camera_info_url", "");
    this->declare_parameter<std::string>("right_camera_info_url", "");

    // Get parameters
    this->get_parameter("camera_device", camera_device_);
    this->get_parameter("image_width", image_width_);
    this->get_parameter("image_height", image_height_);
    this->get_parameter("frame_rate", frame_rate_);
    this->get_parameter("left_topic", left_topic_);
    this->get_parameter("right_topic", right_topic_);
    this->get_parameter("left_frame_id", left_frame_id_);
    this->get_parameter("right_frame_id", right_frame_id_);
    this->get_parameter("exposure", exposure_);
    this->get_parameter("gain", gain_);
    this->get_parameter("brightness", brightness_);
    this->get_parameter("backlight_compensation", backlight_compensation_);
    this->get_parameter("sharpness", sharpness_);
    this->get_parameter("white_balance_automatic", white_balance_automatic_);
    this->get_parameter("contrast", contrast_);
    this->get_parameter("left_camera_info_url", left_camera_info_url_);
    this->get_parameter("right_camera_info_url", right_camera_info_url_);

    // Create publishers
    left_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(left_topic_, 10);
    right_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(right_topic_, 10);
    left_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(left_topic_ + "_info", 10);
    right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(right_topic_ + "_info", 10);

    // Initialize camera info managers
    left_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
        this, "stereo_left", left_camera_info_url_);
    right_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
        this, "stereo_right", right_camera_info_url_);

    RCLCPP_INFO(this->get_logger(), "Stereo Camera Publisher initialized");
}

StereoCameraPublisher::~StereoCameraPublisher()
{
    if (camera_.isOpened()) {
        camera_.release();
    }
}

bool StereoCameraPublisher::initialize()
{
    // Extract camera index from device path (e.g., /dev/video2 -> 2)
    int camera_index = 2;
    if (camera_device_.find("/dev/video") == 0) {
        camera_index = std::stoi(camera_device_.substr(10));
    }

    // Open camera with V4L2 backend and MJPEG
    camera_.open(camera_index, cv::CAP_V4L2);

    if (!camera_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera: %s", camera_device_.c_str());
        return false;
    }

    // Set camera properties
    camera_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    camera_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
    camera_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
    camera_.set(cv::CAP_PROP_FPS, frame_rate_);

    // Set camera controls using v4l2-ctl for better control
    // Note: OpenCV's control setting is limited, so we use system commands
    std::string v4l2_cmd = "v4l2-ctl -d " + camera_device_ +
                          " --set-ctrl=auto_exposure=1" +  // Manual exposure mode
                          " --set-ctrl=exposure_time_absolute=" + std::to_string(exposure_) +
                          " --set-ctrl=gain=" + std::to_string(gain_) +
                          " --set-ctrl=brightness=" + std::to_string(brightness_) +
                          " --set-ctrl=backlight_compensation=" + std::to_string(backlight_compensation_) +
                          " --set-ctrl=sharpness=" + std::to_string(sharpness_) +
                          " --set-ctrl=white_balance_automatic=" + std::to_string(white_balance_automatic_) +
                          " --set-ctrl=contrast=" + std::to_string(contrast_);

    RCLCPP_INFO(this->get_logger(), "Executing: %s", v4l2_cmd.c_str());
    int ret = system(v4l2_cmd.c_str());
    if (ret == 0) {
        RCLCPP_INFO(this->get_logger(), "Camera controls set successfully");
        RCLCPP_INFO(this->get_logger(), "  auto_exposure=1 (manual)");
        RCLCPP_INFO(this->get_logger(), "  exposure_time_absolute=%d", exposure_);
        RCLCPP_INFO(this->get_logger(), "  gain=%d", gain_);
        RCLCPP_INFO(this->get_logger(), "  brightness=%d", brightness_);
        RCLCPP_INFO(this->get_logger(), "  backlight_compensation=%d", backlight_compensation_);
        RCLCPP_INFO(this->get_logger(), "  sharpness=%d", sharpness_);
        RCLCPP_INFO(this->get_logger(), "  white_balance_automatic=%d", white_balance_automatic_);
        RCLCPP_INFO(this->get_logger(), "  contrast=%d", contrast_);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to set camera controls (exit code: %d)", ret);
        RCLCPP_ERROR(this->get_logger(), "Command was: %s", v4l2_cmd.c_str());
    }

    // Get actual camera settings
    int actual_width = camera_.get(cv::CAP_PROP_FRAME_WIDTH);
    int actual_height = camera_.get(cv::CAP_PROP_FRAME_HEIGHT);
    double actual_fps = camera_.get(cv::CAP_PROP_FPS);

    cameras_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Stereo camera initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Device: %s (index %d)", camera_device_.c_str(), camera_index);
    RCLCPP_INFO(this->get_logger(), "Full resolution: %dx%d (side-by-side)", actual_width, actual_height);
    RCLCPP_INFO(this->get_logger(), "Per-eye resolution: %dx%d", actual_width / 2, actual_height);
    RCLCPP_INFO(this->get_logger(), "Frame rate: %.1f Hz", actual_fps);

    return true;
}

void StereoCameraPublisher::run()
{
    if (!cameras_initialized_) {
        RCLCPP_ERROR(this->get_logger(), "Cameras not initialized");
        return;
    }

    // Create timer for periodic publishing
    auto period = std::chrono::duration<double>(1.0 / frame_rate_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&StereoCameraPublisher::captureAndPublish, this));

    RCLCPP_INFO(this->get_logger(), "Publishing stereo images at %.1f Hz", frame_rate_);
}

void StereoCameraPublisher::captureAndPublish()
{
    cv::Mat full_frame;

    // Capture frame from camera
    if (!camera_.read(full_frame) || full_frame.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Failed to capture frame from camera");
        return;
    }

    // Get timestamp for both images
    auto timestamp = this->now();

    // Split side-by-side image into left and right
    int half_width = full_frame.cols / 2;
    cv::Mat left_frame = full_frame(cv::Rect(0, 0, half_width, full_frame.rows));
    cv::Mat right_frame = full_frame(cv::Rect(half_width, 0, half_width, full_frame.rows));

    // Convert left frame to ROS message
    auto left_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_frame).toImageMsg();
    left_msg->header.stamp = timestamp;
    left_msg->header.frame_id = left_frame_id_;
    left_image_pub_->publish(*left_msg);

    // Publish left camera info
    auto left_info = left_info_manager_->getCameraInfo();
    left_info.header = left_msg->header;
    left_info.width = half_width;
    left_info.height = full_frame.rows;
    left_info_pub_->publish(left_info);

    // Convert right frame to ROS message
    auto right_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_frame).toImageMsg();
    right_msg->header.stamp = timestamp;
    right_msg->header.frame_id = right_frame_id_;
    right_image_pub_->publish(*right_msg);

    // Publish right camera info
    auto right_info = right_info_manager_->getCameraInfo();
    right_info.header = right_msg->header;
    right_info.width = half_width;
    right_info.height = full_frame.rows;
    right_info_pub_->publish(right_info);
}
