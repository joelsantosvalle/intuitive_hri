#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <map>
#include <vector>
#include <thread>
#include <sensor_msgs/msg/joint_state.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string>
#include "robotiq_driver/robotiq_gripper_interface.hpp"

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

// Action Manager Class
class ActionManager : public rclcpp::Node {
public:
    ActionManager() : Node("ActionManager"), ur3_controller_(new UR3Controller(this)) {
        setup_subscribers();
        initialize_commands();
        configure_gripper();
    }

private:
    std::shared_ptr<UR3Controller> ur3_controller_;
    std::map<std::string, std::vector<double>> command_map_;
    bool object_in_gripper = false;
    
    void setup_subscribers() {
        hand_state_sub_ = this->create_subscription<std_msgs::msg::Float32>("hand_state", 1, 
                        std::bind(&ActionManager::handStateCallback, this, std::placeholders::_1));
        robot_position_sub_ = this->create_subscription<geometry_msgs::msg::Point>("robot_position", 5, 
                        std::bind(&ActionManager::robotPositionCallback, this, std::placeholders::_1));
        // Other necessary subscribers can be initialized here
    }

    void initialize_commands() {
        command_map_["leftapproach"] = {-1.588, -1.25, -0.98, -1.04, 1.58, 0};
        command_map_["lefttop"] = {-1.66, -1.08, -1.48, -0.91, 1.61, 0};
        command_map_["rightapproach"] = {1.56, -2.04, 0.934, -1.73, -1.56, 0};
        command_map_["home"] = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0};
    }

    void configure_gripper() {
        gripper.deactivateGripper();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        gripper.activateGripper();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        gripper.setGripperPosition(0x00);
        ur3_controller_->execute_home_pose();
    }

    // Callbacks
    void handStateCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        if (msg->data > 0.5) { 
            if (!object_in_gripper) {
                ur3_controller_->pick_object_from_table();
                object_in_gripper = true;
            }
        } else if (object_in_gripper) {
            ur3_controller_->drop_object_to_table();
            object_in_gripper = false;
        }
    }

    void robotPositionCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        // Implement behavior based on robot position here
    }
};

// Main
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
