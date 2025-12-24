#include "stereo_camera_publisher/stereo_camera_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<StereoCameraPublisher>();

    if (!node->initialize()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to initialize stereo camera node");
        rclcpp::shutdown();
        return 1;
    }

    node->run();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
