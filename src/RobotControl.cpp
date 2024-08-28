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

//add 30 mm from position of gripper  in z and 50 mm along the y

// Command structure to hold position and joint angles, position in meters and orientation in radians
struct Command {
    geometry_msgs::msg::Vector3 position;  // Dynamic x, y, z values
    geometry_msgs::msg::Vector3 orientation;      // Joint angles
};

// Command structure to hold position and joint angles, the joint angles passed to this function should be in radians not degrees
struct CommandJoints {
    std::vector<double> joint_angles;      // Vector to hold 6 joint angles

    // Constructor to initialize the joint_angles vector with 6 elements
    CommandJoints() : joint_angles(6, 0.0) {}
};

constexpr auto kComPort = "/tmp/ttyUR";
constexpr auto kSlaveID = 0x09;
RobotiqGripperInterface gripper(kComPort, kSlaveID);

//Class for communication with the UR3

class UR3Controller {
public:
    UR3Controller(rclcpp::Node* node)
        : node_(node) {
        script_command_pub_ = node_->create_publisher<std_msgs::msg::String>("/urscript_interface/script_command", 10);
    }

    void pick_object_from_table() {
        std_msgs::msg::String msg;
        msg.data = "def my_prog():     movej(["  + std::to_string(-1.07) +"," +std::to_string(-0.747)+"," +std::to_string(0.67)+"," +std::to_string(-1.56)+"," +std::to_string(-1.51)+","+ std::to_string(0.62) + "], a=0.14, v=0.30, r=0.01) "+" movej(["  + std::to_string(-1.0492) +"," +std::to_string(-0.586)+"," +std::to_string(0.685)+"," +std::to_string(-1.614)+"," +std::to_string(-1.506)+","+ std::to_string(0.495) + "], a=0.12, v=0.25, r=0.01)\nend"; 
        script_command_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(10));
        std::cout << "Closing gripper...\n" << std::endl;
        gripper.setGripperPosition(0xFF);
        rclcpp::sleep_for(std::chrono::seconds(4));
        msg.data = "def my_prog():     movej(["  + std::to_string(-1.07) +"," +std::to_string(-0.747)+"," +std::to_string(0.67)+"," +std::to_string(-1.56)+"," +std::to_string(-1.51)+","+ std::to_string(0.62) + "], a=0.12, v=0.25, r=0.01) "+" movej(["  + std::to_string(0) +"," +std::to_string(-1.57)+"," +std::to_string(0)+"," +std::to_string(-1.57)+"," +std::to_string(0)+","+ std::to_string(0) + "], a=0.12, v=0.25, r=0.01)\nend"; 
        script_command_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(10));
    }

    void drop_object_to_table() {
        std_msgs::msg::String msg;
        msg.data = "def my_prog():     movej(["  + std::to_string(-1.07) +"," +std::to_string(-0.747)+"," +std::to_string(0.67)+"," +std::to_string(-1.56)+"," +std::to_string(-1.51)+","+ std::to_string(0.62) + "], a=0.14, v=0.30, r=0.01) "+" movej(["  + std::to_string(-1.0492) +"," +std::to_string(-0.586)+"," +std::to_string(0.685)+"," +std::to_string(-1.614)+"," +std::to_string(-1.506)+","+ std::to_string(0.495) + "], a=0.12, v=0.25, r=0.01)\nend"; 
        script_command_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(10));
        gripper.setGripperPosition(0x00);
        rclcpp::sleep_for(std::chrono::seconds(4));
        msg.data = "def my_prog():     movej(["  + std::to_string(-1.07) +"," +std::to_string(-0.747)+"," +std::to_string(0.67)+"," +std::to_string(-1.56)+"," +std::to_string(-1.51)+","+ std::to_string(0.62) + "], a=0.12, v=0.25, r=0.01) "+" movej(["  + std::to_string(0) +"," +std::to_string(-1.57)+"," +std::to_string(0)+"," +std::to_string(-1.57)+"," +std::to_string(0)+","+ std::to_string(0) + "], a=0.12, v=0.25, r=0.01)\nend"; 
        script_command_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(10));
    }

    void execute_command(Command command) {
        std_msgs::msg::String msg;
        msg.data = " movej(p["  + std::to_string(command.position.x) +"," +std::to_string(command.position.y)+"," +std::to_string(command.position.z)+"," +std::to_string(command.orientation.x)+"," +std::to_string(command.orientation.y)+","+ std::to_string(command.orientation.z) + "], a=1.2, v=0.25, r=0.01)"; 
        script_command_pub_->publish(msg);
    }

    void execute_command_joints(CommandJoints command) {
        std_msgs::msg::String msg;
        msg.data = " movej(p["  + std::to_string(command.joint_angles[0]) +"," +std::to_string(command.joint_angles[1])+"," +std::to_string(command.joint_angles[2])+"," +std::to_string(command.joint_angles[3])+"," +std::to_string(command.joint_angles[4])+","+ std::to_string(command.joint_angles[5]) + "], a=1.2, v=0.25, r=0.01)"; 
        script_command_pub_->publish(msg);
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr script_command_pub_;
};

//Class for choosing action to be done 

class ActionManager : public rclcpp::Node
{
public:
    ActionManager() : Node("ActionManager"),ur3_controller_(new UR3Controller(this)) ,command_key(" "), offset(0.75), object_in_gripper(false), picked_(false), placed_(false)
    {
        setup_subscribers();
        initialize_commands();
        std::cout << "Deactivating gripper...\n";
        gripper.deactivateGripper();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Activating gripper...Success\n";
        gripper.activateGripper();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        std::cout << "Opening gripper...\n" << std::endl ;
        gripper.setGripperPosition(0x00);
         
        // Set up a timer to periodically call evaluate_conditions_and_act
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(3000),  // Timer period
            std::bind(&ActionManager::evaluate_conditions_and_act, this));
    }

private:
    std::map<std::string, std::vector<double>> command_map_;
    std::shared_ptr<UR3Controller> ur3_controller_;
    std::string command_key;
    float hand_state_;
    float hand_normal_;
    geometry_msgs::msg::Point robot_position_;
    geometry_msgs::msg::Point old_robot_position_;
    float hand_time_;
    geometry_msgs::msg::Point hand_rate_of_change_;
    float offset;
    bool object_in_gripper;
    rclcpp::TimerBase::SharedPtr timer_;
    bool picked_, placed_;
    sensor_msgs::msg::JointState joint_states;
    
     // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hand_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hand_normal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr robot_position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hand_time_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr hand_rate_of_change_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    void setup_subscribers() {
        // Subscriber to hand state
        hand_state_sub_ = this->create_subscription<std_msgs::msg::Float32>("hand_state", 1, std::bind(&ActionManager::handStateCallback, this, std::placeholders::_1));

        // Subscriber to hand normal
        hand_normal_sub_ = this->create_subscription<std_msgs::msg::Float32>("hand_normal", 1, std::bind(&ActionManager::handNormalCallback, this, std::placeholders::_1));

        // Subscriber to hand position
        robot_position_sub_ = this->create_subscription<geometry_msgs::msg::Point>("robot_position", 5, std::bind(&ActionManager::handPositionCallback, this, std::placeholders::_1));

        // Subscriber to hand time
        hand_time_sub_ = this->create_subscription<std_msgs::msg::Float32>("hand_time", 1, std::bind(&ActionManager::handTimeCallback, this, std::placeholders::_1));

        // Subscriber to hand rate of change
        hand_rate_of_change_sub_ = this->create_subscription<geometry_msgs::msg::Point>("hand_rate_of_change", 5, std::bind(&ActionManager::handRateOfChangeCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 1, std::bind(&ActionManager::jointStatesCallback, this, std::placeholders::_1));
    }

    void handStateCallback(const std_msgs::msg::Float32::SharedPtr msg) {
            hand_state_ = msg->data;
    }

    void handNormalCallback(const std_msgs::msg::Float32::SharedPtr msg) {
            hand_normal_ = msg->data;
    }

    void handPositionCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        robot_position_ = *msg;
    }

    void handTimeCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        hand_time_ = msg->data;
    }

    void handRateOfChangeCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        hand_rate_of_change_ = *msg;
    }

    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        joint_states = *msg;
    }

    void initialize_commands() {
        // Initialize the command map with predefined joint angles for each command
        command_map_["left"] = {0.0, -1.57, 1.0, 0.0, 0.0, 0.0};
        command_map_["leftapproach"] = {0.0, -1.57, 1.0, 0.0, 0.0, 0.0};
        command_map_["lefttop"] = {0.5, -1.0, 1.5};
        command_map_["leftcenter"] = {0.0, -1.0, 1.0};
        command_map_["leftbottom"] = {0.5, -1.57, 1.0};
        command_map_["right"] = {0.0, -1.57, 0.5, 0.0, 0.0, 0.0};
        command_map_["rightapproach"] = {0.0, -1.57, 0.5, 0.0, 0.0, 0.0};
        command_map_["righttop"] = {0.5, -1.0, 0.5};
        command_map_["rightcenter"] = {0.0, -1.57, 0.5};
        command_map_["rightbottom"] = {0.5, -1.57, 0.5};
        command_map_["home"] = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0};
        command_map_["approachpose"] = {-1.07, -0.747, 0.67, -1.56, -1.51, 0.62};
        command_map_["pickuppose"] = {-1.0492, -0.586, 0.685, -1.614, -1.506, 0.495};
    }

    void evaluate_conditions_and_act() {
        if (should_pick_object_table()) {
            ur3_controller_->pick_object_from_table();
        } else if (should_place_object_table()) {
            ur3_controller_->drop_object_to_table();
        } else if (should_turn() && correct_hand_position()) {
            execute_command(command_key, offset);
        } else if (should_approach_human() && correct_hand_position()) {
            execute_command(command_key, 0.45);
        } else if (should_place_object_operator() && correct_hand_position()) {
            execute_command(command_key, offset);
        } else if (should_change_position() && correct_hand_position()) {
            execute_command_joints(command_key);
        } else {
            maintain_current_state();
        }
    }

    void execute_command(std::string command_key, float offset) {
        // Find the command in the map
        auto command_it = command_map_.find(command_key);
        
        if (command_it != command_map_.end()) {
            // Get the joint angles associated with the command
            std::vector<double> joint_angles = command_it->second;
            
            // Create a new CommandJoints object to hold the position and angles
            Command command_joints;
            
            // Assign the joint angles
            command_joints.orientation.x = joint_angles[0];
            command_joints.orientation.y = joint_angles[1];
            command_joints.orientation.z = joint_angles[2];
            
            // Populate the position with dynamic sensor data
            command_joints.position.x = robot_position_.x;
            command_joints.position.y = robot_position_.y;
            command_joints.position.z = robot_position_.z + offset;
            
            // Execute the command using the UR3 controller
            ur3_controller_->execute_command(command_joints);
        } else {
            RCLCPP_WARN(this->get_logger(), "Command not found: %s", command_key.c_str());
        }
    }

    // Function to execute command using joint angles
    void execute_command_joints(std::string command_key) {
        // Find the command in the map
        auto command_it = command_map_.find(command_key);
        
        if (command_it != command_map_.end()) {
            CommandJoints command_joints;
            command_joints.joint_angles = command_it->second;
            ur3_controller_->execute_command_joints(command_joints);
        } else {
            RCLCPP_WARN(this->get_logger(), "Command not found: %s", command_key.c_str());
        }
    }

    bool correct_hand_position()
    {
        return (hand_state_ >= 0 && hand_state_ <= 0.6) && (hand_time_ > 2) && hand_normal_ > 0; 
    }

    bool should_pick_object_table() {
        if(object_in_gripper == false && hand_time_ > 6 && hand_normal_ > 0 && hand_state_ == 0)
        { 
            object_in_gripper = true;
            picked_ = true;
            return true;
        }
        return false;
    }

    bool should_place_object_table() {
        if(object_in_gripper == true && hand_time_ > 10 && (hand_normal_ < 0 || hand_state_ == 1) && (joint_states.position[0] > -1.59 && joint_states.position[0] < -1.56))
        {
            object_in_gripper = false;
            placed_ = true;
            return true;
        }
        return false;
    }

    bool should_approach_human() {
        return false; 
    }

    bool should_place_object_operator(){
        return false;
    }

    bool should_change_position(){
        return false; 
    }

    bool should_turn(){
        return false; 
    }

    void maintain_current_state() {
        RCLCPP_INFO(this->get_logger(), "Maintaining current state");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionManager>());
    rclcpp::shutdown();
    return 0;
}
