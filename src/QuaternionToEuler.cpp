#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

// Structure to hold the Euler angles
struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles toEulerAngles(const geometry_msgs::msg::Quaternion& q) {
    EulerAngles angles;

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    return angles;
}

// ROS 2 node class
class EulerPublisherNode : public rclcpp::Node {
public:
    EulerPublisherNode() : Node("euler_publisher") {
        // Create publisher for Euler angles
        euler_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("euler_angles", 10);

        // Create subscriber for quaternion topic
        quaternion_subscriber_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "quaternion", 10,
            std::bind(&EulerPublisherNode::quaternionCallback, this, std::placeholders::_1)
        );
    }

private:
    void quaternionCallback(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
        // Convert quaternion to Euler angles
        EulerAngles angles = toEulerAngles(*msg);

        // Prepare the message to publish
        geometry_msgs::msg::Vector3 euler_msg;
        euler_msg.x = angles.roll;
        euler_msg.y = angles.pitch;
        euler_msg.z = angles.yaw;

        RCLCPP_INFO(this->get_logger(), "Publishing Roll: %.2f, Pitch: %.2f, Yaw: %.2f", angles.roll, angles.pitch, angles.yaw);

        // Publish Euler angles
        euler_publisher_->publish(euler_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr euler_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr quaternion_subscriber_;
};