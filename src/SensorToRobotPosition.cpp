#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class SensorToRobotConverter : public rclcpp::Node
{
public:
    SensorToRobotConverter()
        : Node("hand_to_robot_converter")
    {
        // Publisher for robot position
        pub_robot_position_ = this->create_publisher<geometry_msgs::msg::Pose>("robot_position", 10);

        // Subscriber to hand orientation
        sub_hand_orientation_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "hand_orientation_sensor", 10,
            std::bind(&SensorToRobotConverter::orientation_callback, this, std::placeholders::_1));

        // Subscriber to hand position
        sub_palm_position_stable_ = this->create_subscription<geometry_msgs::msg::Point>(
            "hand_position_sensor", 10,
            std::bind(&SensorToRobotConverter::position_callback, this, std::placeholders::_1));
    }

private:
    // Callback to handle the orientation message
    void orientation_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg)
    {
        // Convert Quaternion to Roll, Pitch, Yaw
        tf2::Quaternion q(
            msg->x,
            msg->y,
            msg->z,
            msg->w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Update the pose message with roll, pitch, yaw as the orientation
        robot_pose_.orientation.x = roll;
        robot_pose_.orientation.y = pitch;
        robot_pose_.orientation.z = yaw;
    }

    // Callback to handle the position message
    void position_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        // Update the pose message with the position
        robot_pose_.position = *msg;

        // Publish the updated pose
        pub_robot_position_->publish(robot_pose_);
    }

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_robot_position_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_hand_orientation_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_palm_position_stable_;

    // Robot pose to be published
    geometry_msgs::msg::Pose robot_pose_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorToRobotConverter>());
    rclcpp::shutdown();
    return 0;
}
