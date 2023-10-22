#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <memory>
#include <math.h>

#include "libHCSR04_ros/UltrasonicHCSR04Wrapper.hpp"


UltrasonicHCSR04Wrapper::UltrasonicHCSR04Wrapper() : Node("HCSR04_ultrasonic_driver")
{
    this->declare_parameter<int>("minimum_range", 0.03);        // m
    this->declare_parameter<int>("maximum_range", 4.0);         // m
    this->declare_parameter<double>("field_of_view_deg", 15.0); // measuring sensor angle in deg
    this->declare_parameter<int>("trigger_pin", 14);
    this->declare_parameter<int>("echo_pin", 4);
    this->declare_parameter<int>("relative_to_frame_id", "base_link");

    min_range_ = this->get_parameter("minimum_range").as_int();
    max_range_ = this->get_parameter("maximum_range").as_int();
    fovDeg_ = this->get_parameter("field_of_view_deg").as_double();
    reference_frame_ = this->get_parameter("relative_to_frame_id");

    trigger_ = this->get_parameter("trigger_pin").as_int();
    echo_ = this->get_parameter("echo_pin").as_int();

    ultrasonic_ = std::make_unique<HCSR04>(trigger_, echo_);

    current_distance_publisher_ = this->create_publisher<sensor_msgs::msg::Range>("/HCSR04_ultrasonic/distance", 10);
    current_velocity_publisher_ = this->create_publisher<sensor_msgs::msg::Range>("/HCSR04_ultrasonic/relative_velocity", 10);

    current_distance_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&UltrasonicHCSR04Wrapper::publishCurrentDistance, this));
    current_velocity_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&UltrasonicHCSR04Wrapper::publishCurrentVelocity, this));
}

void UltrasonicHCSR04Wrapper::publishCurrentDistance()
{
    double distance = ultrasonic_->distance();

    auto distanceMsg = sensor_msgs::msg::Range();
    distanceMsg.range = distance;
    distanceMsg.header.stamp = this->now();
    distanceMsg.header.frame_id = reference_frame_;
    distanceMsg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    distanceMsg.field_of_view = fovDeg_ * (M_PI / 180);  // rad
    distanceMsg.min_range = min_range_;
    distanceMsg.max_range = max_range_;

    current_distance_publisher_->publish(distanceMsg);
}

void UltrasonicHCSR04Wrapper::publishCurrentVelocity()
{
    double speed = ultrasonic_->speed();

    auto velocityMsg = sensor_msgs::msg::Range();
    velocityMsg.range = speed;
    velocityMsg.header.stamp = this->now();
    velocityMsg.header.frame_id = reference_frame_;
    velocityMsg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    velocityMsg.field_of_view = fovDeg_ * (M_PI / 180); // rad
    velocityMsg.min_range = min_range_;
    velocityMsg.max_range = max_range_;

    current_velocity_publisher_->publish(velocityMsg);
}
