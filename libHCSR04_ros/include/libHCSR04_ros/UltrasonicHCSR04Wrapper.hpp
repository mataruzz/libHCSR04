#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <math.h>

#include <sensor_msgs/msg/range.hpp>

#include "libHCSR04/HCSR04_sensor.hpp"

class UltrasonicHCSR04Wrapper : public rclcpp::Node
{
public:
    UltrasonicHCSR04Wrapper();

    void publishCurrentDistance();

    void publishCurrentVelocity();

private:
    std::unique_ptr<HCSR04> ultrasonic_;
    int trigger_;
    int echo_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr current_distance_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr current_velocity_publisher_;
    rclcpp::TimerBase::SharedPtr current_distance_timer_;
    rclcpp::TimerBase::SharedPtr current_velocity_timer_;
    int min_range_;
    int max_range_;
    double fovDeg_;
    std::string reference_frame_;
};
