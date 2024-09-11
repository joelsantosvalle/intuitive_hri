#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

class SensorToRobotConverter : public rclcpp::Node
{
public:
    SensorToRobotConverter()
        : Node("hand_to_robot_converter")
    {
        // Publisher for robot position
        pub_robot_position_ = this->create_publisher<geometry_msgs::msg::Point>("robot_position", 10);
        pub_robot_orientation_ = this->create_publisher<geometry_msgs::msg::Vector3>("robot_orientation", 10);

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
        /*
        // Step 1: Convert the incoming geometry_msgs::msg::Quaternion to tf2::Quaternion
        tf2::Quaternion sensor_quat;
        tf2::fromMsg(*msg, sensor_quat);

        // Step 2: Convert quaternion to RPY (roll, pitch, yaw)
        double roll, pitch, yaw;
        tf2::Matrix3x3(sensor_quat).getRPY(roll, pitch, yaw);

        std::cout << roll << " " << pitch << " " << yaw << std::endl;  

        tf2::Quaternion rotation_x(tf2::Vector3(1.0, 0.0, 0.0), -M_PI_2);
        tf2::Quaternion rotation_y(tf2::Vector3(0.0, 1.0, 0.0), M_PI);

        tf2::Quaternion rotation = rotation_x * rotation_y;

        tf2::Quaternion robot_quat = rotation * sensor_quat;

        // Step 5: Normalize the resulting quaternion (good practice)
        robot_quat.normalize();

        // Step 6: Convert the quaternion to axis-angle representation
        tf2::Vector3 axis = robot_quat.getAxis();
        double angle = robot_quat.getAngle();

        // Step 7: Compute the rotation components Rx, Ry, Rz for the robot
        double Rx = axis.x() * angle;
        double Ry = axis.y() * angle;
        double Rz = axis.z() * angle;

        robot_pose_.x = Rz;
        robot_pose_.y = Ry;
        robot_pose_.z = Rx;
        */
    }

    // Callback to handle the position message
    void position_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        // Update the pose message with the position
        robot_pose_.x = msg->x;
        robot_pose_.y = ((-1*(msg->z)) - 0.38 - 0.02);
        robot_pose_.z = msg->y;

        // Publish the updated pose
        pub_robot_position_->publish(robot_pose_);
    }

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_robot_position_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_robot_orientation_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_hand_orientation_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_palm_position_stable_;

    // Robot pose to be published
    geometry_msgs::msg::Point robot_pose_;

    // Robot orientation to be published
    geometry_msgs::msg::Point robot_orientation_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorToRobotConverter>());
    rclcpp::shutdown();
    return 0;
}
