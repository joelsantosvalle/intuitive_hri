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