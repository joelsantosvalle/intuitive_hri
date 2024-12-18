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

// Command structure to hold position and joint angles, position in meters and orientation in radians
struct Command {
    geometry_msgs::msg::Vector3 position;  // Dynamic x, y, z values
    geometry_msgs::msg::Vector3 orientation;      // Joint angles
};

// Command structure to hold position and joint angles, the joint angles passed to this function should be in radians not degrees
struct CommandJoints {
    std::vector<double> joint_angles;      // Vector to hold 6 joint angles

    // Constructor to initialize the joint_angles vector with 6 elements
    CommandJoints() { joint_angles = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0}; }
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
        msg.data = "def my_prog():     movej(["  + std::to_string(-1.07) +"," +std::to_string(-0.747)+"," +std::to_string(0.67)+"," +std::to_string(-1.56)+"," +std::to_string(-1.51)+","+ std::to_string(0.62) + "], a=0.22, v=0.35, r=0.01) "+" movej(["  + std::to_string(-1.0492) +"," +std::to_string(-0.586)+"," +std::to_string(0.685)+"," +std::to_string(-1.614)+"," +std::to_string(-1.506)+","+ std::to_string(0.495) + "], a=0.22, v=0.35, r=0.01)\nend"; 
        script_command_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(8));
        gripper.setGripperPosition(0xFF);
        rclcpp::sleep_for(std::chrono::seconds(2));
        msg.data = "def my_prog():     movej(["  + std::to_string(-1.07) +"," +std::to_string(-0.747)+"," +std::to_string(0.67)+"," +std::to_string(-1.56)+"," +std::to_string(-1.51)+","+ std::to_string(0.62) + "], a=0.22, v=0.35, r=0.01) "+" movej(["  + std::to_string(0) +"," +std::to_string(-1.57)+"," +std::to_string(0)+"," +std::to_string(-1.57)+"," +std::to_string(0)+","+ std::to_string(0) + "], a=0.22, v=0.35, r=0.01)\nend"; 
        script_command_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(8));
    }

    void drop_object_to_table() {
        std_msgs::msg::String msg;
        msg.data = "def my_prog():     movej(["  + std::to_string(-1.07) +"," +std::to_string(-0.747)+"," +std::to_string(0.67)+"," +std::to_string(-1.56)+"," +std::to_string(-1.51)+","+ std::to_string(0.62) + "], a=0.22, v=0.35, r=0.01) "+" movej(["  + std::to_string(-1.0492) +"," +std::to_string(-0.586)+"," +std::to_string(0.685)+"," +std::to_string(-1.614)+"," +std::to_string(-1.506)+","+ std::to_string(0.495) + "], a=0.22, v=0.35, r=0.01)\nend"; 
        script_command_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(8));
        gripper.setGripperPosition(0x00);
        rclcpp::sleep_for(std::chrono::seconds(2));
        msg.data = "def my_prog():     movej(["  + std::to_string(-1.07) +"," +std::to_string(-0.747)+"," +std::to_string(0.67)+"," +std::to_string(-1.56)+"," +std::to_string(-1.51)+","+ std::to_string(0.62) + "], a=0.22, v=0.35, r=0.01) "+" movej(["  + std::to_string(0) +"," +std::to_string(-1.57)+"," +std::to_string(0)+"," +std::to_string(-1.57)+"," +std::to_string(0)+","+ std::to_string(0) + "], a=0.22, v=0.35, r=0.01)\nend"; 
        script_command_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(8));
    }

    void execute_command(Command command) {
        std_msgs::msg::String msg;
        msg.data = "def my_prog():     movej(p["  + std::to_string(command.position.x) +"," +std::to_string(command.position.y)+"," +std::to_string(command.position.z)+"," +std::to_string(command.orientation.x)+"," +std::to_string(command.orientation.y)+","+ std::to_string(command.orientation.z) + "], a=0.2, v=0.3, r=0)\nend"; 
        script_command_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(6));
    }

    void execute_home_pose() {
        std_msgs::msg::String msg;
        msg.data = "movej([0, -1.571, 0, -1.571, 0, 0], a=0.12, v=0.25, r=0)"; 
        script_command_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(5));
    }

    void execute_command_joints(CommandJoints command) {
        std_msgs::msg::String msg;
        msg.data = "def my_prog():     movej(["  + std::to_string(command.joint_angles[0]) +"," +std::to_string(command.joint_angles[1])+"," +std::to_string(command.joint_angles[2])+"," +std::to_string(command.joint_angles[3])+"," +std::to_string(command.joint_angles[4])+","+ std::to_string(command.joint_angles[5]) + "], a=0.24, v=0.45, r=0)\nend"; 
        script_command_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(6));
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr script_command_pub_;
};

//Class for choosing action to be done 

class ActionManager : public rclcpp::Node
{
public:
    ActionManager() : Node("ActionManager"),ur3_controller_(new UR3Controller(this)) ,command_key(" "), offset(0.047), object_in_gripper(false), picked_(false), placed_(false), approach(false), moving(false)
    {
        setup_subscribers();
        initialize_commands();
        command_key_vec.resize(3);
        gripper.deactivateGripper();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        gripper.activateGripper();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        gripper.setGripperPosition(0x00);
        ur3_controller_->execute_home_pose();
         
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
    bool approach;
    std::vector<std::string> command_key_vec;
    std::string location;
    std::string last_location;
    float old_hand_id;
    float hand_id_;
    bool moving;
    
     // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hand_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hand_normal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr robot_position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hand_time_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr hand_rate_of_change_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hand_id_sub_;

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

        hand_id_sub_ = this->create_subscription<std_msgs::msg::Float32>("hand_id", 1, std::bind(&ActionManager::handidCallback, this, std::placeholders::_1));
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

    void handidCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        hand_id_ = msg->data;
    }

    void initialize_commands() {
        // Initialize the command map with predefined joint angles for each command
        command_map_["leftapproach"] = {-1.588, -1.25, -0.98, -1.04, 1.58, 0};
        command_map_["lefttop"] = {-1.66, -1.08, -1.48, -0.91, 1.61, 0};
        command_map_["leftcenter"] = {-1.71, -1.204, -2.08, -0.91, 1.61, 0};
        command_map_["leftbottom"] = {-1.71, -1.23, -1.72, -0.91, 1.61, 0};
        command_map_["topangles"] = {1.371, -1.332, 1.011};
        command_map_["centerangles"] = {1.660, -1.720, 0.719};
        command_map_["bottomangles"] = {1.760, -1.541, 0.800};
        command_map_["rightapproach"] = {1.56, -2.04, 0.934, -1.73, -1.56, 0};
        command_map_["righttop"] = {1.74, -2.012, 1.46, -2.19, -1.5, 0};
        command_map_["rightcenter"] = {1.54, -2.23, 1.67, -1.83, -1.5, 0};
        command_map_["rightbottom"] = {1.54, -1.94, 1.74, -2.19, -1.5, 0};
        command_map_["home"] = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0};
    }

    int kbhit() {
        struct termios oldt, newt;
        int ch;
        int oldf;

        // Get terminal settings and make them non-blocking
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

        // Check if key is pressed
        ch = getchar();

        // Reset terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);

        if(ch != EOF) {
            ungetc(ch, stdin);
            return 1;
        }

        return 0;
    }

    void evaluate_conditions_and_act() {

        // Check if 'Enter' is pressed
        if (kbhit()) {
            char c = getchar();
            if (c == '\n' && object_in_gripper == false) {
                ur3_controller_->pick_object_from_table();
                object_in_gripper = true;
                picked_ = true;
            }
            else if(c == '\n' && object_in_gripper == true)
            {
                 ur3_controller_->drop_object_to_table();
                 object_in_gripper = false;
                 picked_ = false;
            }
        } else if (correct_hand_position() && picked_ && object_in_gripper) {
            if((should_approach_human() && (joint_states.position[0] > -1.59 && joint_states.position[0] < -1.56 && hand_time_ > 1)) || ((robot_position_.x > old_robot_position_ .x && robot_position_.x > 0 && last_location == "left") || (robot_position_.x < old_robot_position_.x && robot_position_.x < 0 && last_location == "right")))
            {
                 execute_command_joints(command_key_vec[2]);
                 old_robot_position_.x = robot_position_.x;
                 old_robot_position_.y = robot_position_.y;
                 old_robot_position_.z = robot_position_.z;
                 last_location = (robot_position_.x > 0)? "right" : "left";
            }
            if (should_place_object_operator() && correct_hand_position() && picked_) {
                execute_command(command_key, offset);
                gripper.setGripperPosition(0x00);
                rclcpp::sleep_for(std::chrono::seconds(1));
                placed_ = true;
                object_in_gripper = false;
            }
        }  

        if (picked_ && placed_) {
            picked_ = false;
            placed_ = false;
            ur3_controller_->execute_home_pose();
            hand_id_ = old_hand_id;
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

    bool should_approach_human() {
        command_key_vec[0] = "home";
        if(hand_normal_ > 0 && hand_state_ == 0 && object_in_gripper == true && placed_ == false && hand_time_ > 1 && robot_position_.x < 0)
        {
            location = "left";
            command_key_vec[1] = "leftapproach";
            if(robot_position_.z >= 0.31 && robot_position_.z < 0.51)
            {
                command_key_vec[2] = "lefttop";
                approach = true;
                return true;
            }
            else if(robot_position_.z >= 0.21 && robot_position_.z < 0.31)
            {
                command_key_vec[2] = "leftcenter";
                approach = true;
                return true;
            }
            else if(robot_position_.z >= 0.05 && robot_position_.z < 0.21)
            {
                command_key_vec[2] = "leftbottom";
                approach = true;
                return true;
            }
            else{
                return false;
            }
        }
        else if(hand_normal_ > 0 && hand_state_ == 0 && object_in_gripper == true && placed_ == false && hand_time_ > 1 && robot_position_.x > 0)
        {
            location = "right";
            command_key_vec[1] = "rightapproach";
            if(robot_position_.z >= 0.31 && robot_position_.z < 0.51)
            {
                command_key_vec[2] = "righttop";
                approach = true;
                return true;
            }
            else if(robot_position_.z >= 0.21 && robot_position_.z < 0.31)
            {
                command_key_vec[2] = "rightcenter";
                approach = true;
                return true;
            }
            else if(robot_position_.z >= 0.05 && robot_position_.z < 0.21)
            {
                command_key_vec[2] = "rightbottom";
                approach = true;
                return true;
            }
            else{
                return false;
            }
        }
        return false; 
    }

    bool should_place_object_operator(){
        if(hand_normal_ > 0 && hand_state_ == 0 && object_in_gripper == true && (hand_time_ > 2) && approach && ((location == "left" && robot_position_.x < 0) || (location == "right" && robot_position_.x > 0)) && (hand_rate_of_change_.x <= 10 && hand_rate_of_change_.x >= -10 && hand_rate_of_change_.y <= 10 && hand_rate_of_change_.y >= -10 && hand_rate_of_change_.z <= 10 && hand_rate_of_change_.z >= -10))
        {
            if(robot_position_.z >= 0.31 && robot_position_.z < 0.51)
            {
                command_key = "topangles";
                return true;
            }
            else if(robot_position_.z >= 0.21 && robot_position_.z < 0.31)
            {
                command_key = "centerangles";
                return true;
            }
            else if(robot_position_.z >= 0.05 && robot_position_.z < 0.21)
            {
                command_key = "bottomangles";
                return true;
            }
        }
        approach = false;
        return false;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionManager>());
    rclcpp::shutdown();
    return 0;
}