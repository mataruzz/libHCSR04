#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "libHCSR04_ros/UltrasonicHCSR04Wrapper.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Using GPIO convention for pins declaration
    if (wiringPiSetupGpio() == -1)
        return -1;

    auto node = std::make_shared<UltrasonicHCSR04Wrapper>();
    RCLCPP_INFO(node->get_logger(), "Ultrasonic driver is now started");
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
    }

    RCLCPP_INFO(node->get_logger(), "Ultrasonic driver is now shutting down");

    rclcpp::shutdown();
    return 0;
}