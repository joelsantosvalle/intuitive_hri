#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <map>
#include <vector>
#include <sstream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <string>
#include "robotiq_driver/robotiq_gripper_interface.hpp"
#include <thread>
#include <sensor_msgs/msg/joint_state.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// Constant definitions
constexpr auto kComPort = "/tmp/ttyUR";
constexpr auto kSlaveID = 0x09;

// Command structures
struct Command {
    geometry_msgs::msg::Vector3 position;
    geometry_msgs::msg::Vector3 orientation;
};

struct CommandJoints {
    std::vector<double> joint_angles;
    CommandJoints() { joint_angles = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0}; }
};

// UR3 Controller Class
class UR3Controller {
public:
    explicit UR3Controller(rclcpp::Node* node) : node_(node) {
        script_command_pub_ = node_->create_publisher<std_msgs::msg::String>("/urscript_interface/script_command", 10);
        gripper_ = RobotiqGripperInterface(kComPort, kSlaveID);
    }

    void pick_object_from_table() { move_gripper(true); }
    void drop_object_to_table() { move_gripper(false); }
    
    void execute_command(Command command, int time) {
        publish_script_command(command, time);
    }
    
    void execute_home_pose() {
        publish_home_pose();
    }

private:
    rclcpp::Node* node_;
    RobotiqGripperInterface gripper_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr script_command_pub_;

    void publish_script_command(const Command& command, int time) {
        std_msgs::msg::String msg;
        msg.data = "def my_prog(): movej(p[" + std::to_string(command.position.x) + "," +
                   std::to_string(command.position.y) + "," + std::to_string(command.position.z) + "," +
                   std::to_string(command.orientation.x) + "," + std::to_string(command.orientation.y) + "," +
                   std::to_string(command.orientation.z) + "], a=0.2, v=0.3, r=0.05)\nend";
        script_command_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(time));
    }

    void publish_home_pose() {
        std_msgs::msg::String msg;
        msg.data = "movej([0, -1.571, 0, -1.571, 0, 0], a=0.12, v=0.25, r=0)";
        script_command_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(5));
    }

    void move_gripper(bool pick) {
        gripper_.setGripperPosition(pick ? 0xFF : 0x00);
        rclcpp::sleep_for(std::chrono::seconds(2));
    }
};