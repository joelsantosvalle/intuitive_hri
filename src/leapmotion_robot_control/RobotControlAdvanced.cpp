/**
 * RobotControlAdvanced.cpp
 *
 * @date 19.12.2024
 * @author Joel Santos
 *
 * This program controls a UR3 robotic arm and a Robotiq gripper. It listens to 
 * sensor (Leap Motion) data and decides when to pick up, place, or move 
 * the robotic arm based on that data.
 */

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
 
 /**
  * @struct Command
  * Holds a position (in meters) and an orientation (in radians)
  * for Cartesian moves of the UR3.
  */
 struct Command {
     geometry_msgs::msg::Vector3 position; 
     geometry_msgs::msg::Vector3 orientation;      
 };
 
 /**
  * @struct CommandJoints
  * Holds joint angles (in radians) for the UR3.
  */
 struct CommandJoints {
     std::vector<double> joint_angles;      
     CommandJoints() { joint_angles = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0}; }
 };
 
 constexpr auto kComPort = "/tmp/ttyUR";
 constexpr auto kSlaveID = 0x09;
 RobotiqGripperInterface gripper(kComPort, kSlaveID);
 
 /**
  * @class UR3Controller
  * Manages communication with the UR3 by sending URScript commands.
  */

 class UR3Controller {
 public:
     
    /**
      * @brief Constructor: creates a publisher for URScript commands.
      * @param node Pointer to the parent Node.
      */

     UR3Controller(rclcpp::Node* node)
         : node_(node)
     {
         script_command_pub_ =
             node_->create_publisher<std_msgs::msg::String>("/urscript_interface/script_command", 10);
     }
 
     /**
      * @brief Picks an object from the table using hardcoded positions.
      */
     void pick_object_from_table() {
         std_msgs::msg::String msg;
         msg.data = "def my_prog():     movej(["  + std::to_string(-1.07) +"," 
                    + std::to_string(-0.747)+"," + std::to_string(0.67)+"," 
                    + std::to_string(-1.56)+"," + std::to_string(-1.51)+","+ std::to_string(0.62) 
                    + "], a=0.22, v=0.35, r=0.01) "
                    + " movej([" + std::to_string(-1.0492) +"," +std::to_string(-0.586)+"," 
                    + std::to_string(0.685)+"," + std::to_string(-1.614)+"," 
                    + std::to_string(-1.506)+","+ std::to_string(0.495) 
                    + "], a=0.22, v=0.35, r=0.01)\nend";
         script_command_pub_->publish(msg);
         rclcpp::sleep_for(std::chrono::seconds(8));
 
         gripper.setGripperPosition(0xFF);
         rclcpp::sleep_for(std::chrono::seconds(2));
 
         msg.data = "def my_prog():     movej(["  + std::to_string(-1.07) +"," 
                    + std::to_string(-0.747)+"," + std::to_string(0.67)+"," 
                    + std::to_string(-1.56)+"," + std::to_string(-1.51)+","+ std::to_string(0.62) 
                    + "], a=0.22, v=0.35, r=0.01) "
                    + " movej([" + std::to_string(0) +"," +std::to_string(-1.57)+"," 
                    + std::to_string(0)+"," +std::to_string(-1.57)+"," 
                    + std::to_string(0)+","+ std::to_string(0) 
                    + "], a=0.22, v=0.35, r=0.01)\nend"; 
         script_command_pub_->publish(msg);
         rclcpp::sleep_for(std::chrono::seconds(8));
     }
 
     /**
      * @brief Drops an object onto the table using hardcoded positions.
      */
     void drop_object_to_table() {
         std_msgs::msg::String msg;
         msg.data = "def my_prog():     movej(["  + std::to_string(-1.07) +"," 
                    + std::to_string(-0.747)+"," + std::to_string(0.67)+"," 
                    + std::to_string(-1.56)+"," + std::to_string(-1.51)+","+ std::to_string(0.62) 
                    + "], a=0.22, v=0.35, r=0.01) "
                    + " movej([" + std::to_string(-1.0492) +"," +std::to_string(-0.586)+"," 
                    + std::to_string(0.685)+"," + std::to_string(-1.614)+"," 
                    + std::to_string(-1.506)+","+ std::to_string(0.495) 
                    + "], a=0.22, v=0.35, r=0.01)\nend";
         script_command_pub_->publish(msg);
         rclcpp::sleep_for(std::chrono::seconds(8));
 
         gripper.setGripperPosition(0x00);
         rclcpp::sleep_for(std::chrono::seconds(2));
 
         msg.data = "def my_prog():     movej(["  + std::to_string(-1.07) +"," 
                    + std::to_string(-0.747)+"," + std::to_string(0.67)+"," 
                    + std::to_string(-1.56)+"," + std::to_string(-1.51)+","+ std::to_string(0.62) 
                    + "], a=0.22, v=0.35, r=0.01) "
                    + " movej([" + std::to_string(0) +"," +std::to_string(-1.57)+"," 
                    + std::to_string(0)+"," +std::to_string(-1.57)+"," 
                    + std::to_string(0)+","+ std::to_string(0) 
                    + "], a=0.22, v=0.35, r=0.01)\nend";
         script_command_pub_->publish(msg);
         rclcpp::sleep_for(std::chrono::seconds(8));
     }
 
     /**
      * @brief Moves the UR3 to a predefined "home" pose.
      */
     void execute_home_pose() {
         std_msgs::msg::String msg;
         msg.data = "movej([0, -1.571, 0, -1.571, 0, 0], a=0.12, v=0.25, r=0)"; 
         script_command_pub_->publish(msg);
         rclcpp::sleep_for(std::chrono::seconds(7));
     }
 
     /**
      * @brief Moves the UR3 to a Cartesian position and orientation.
      * @param command The position (x,y,z) and orientation (r,p,y) in radians.
      */
     void execute_command(Command command) {
         std_msgs::msg::String msg;
         msg.data = "def my_prog():     movej(p[" + std::to_string(command.position.x) +"," 
                    + std::to_string(command.position.y)+"," 
                    + std::to_string(command.position.z)+"," 
                    + std::to_string(command.orientation.x)+"," 
                    + std::to_string(command.orientation.y)+"," 
                    + std::to_string(command.orientation.z) 
                    + "], a=0.2, v=0.3, r=0)\nend"; 
         script_command_pub_->publish(msg);
         rclcpp::sleep_for(std::chrono::seconds(6));
     }
 
     /**
      * @brief Moves the UR3 to a set of joint angles.
      * @param command A collection of joint angles in radians.
      */
     void execute_command_joints(CommandJoints command) {
         std_msgs::msg::String msg;
         msg.data = "def my_prog():     movej([" + std::to_string(command.joint_angles[0]) +"," 
                    + std::to_string(command.joint_angles[1])+"," 
                    + std::to_string(command.joint_angles[2])+"," 
                    + std::to_string(command.joint_angles[3])+"," 
                    + std::to_string(command.joint_angles[4])+"," 
                    + std::to_string(command.joint_angles[5]) 
                    + "], a=0.24, v=0.45, r=0)\nend"; 
         script_command_pub_->publish(msg);
         rclcpp::sleep_for(std::chrono::seconds(6));
     }
 
     /**
      * @brief Uses servo commands to move the UR3 in Cartesian space.
      * @param command The Cartesian goal (position + orientation).
      */
     void execute_command_servo(Command command) {
         std_msgs::msg::String msg;
         msg.data = "servoj(get_inverse_kin(p[" + std::to_string(command.position.x) +"," 
                    + std::to_string(command.position.y)+"," 
                    + std::to_string(command.position.z)+"," 
                    + std::to_string(command.orientation.x)+"," 
                    + std::to_string(command.orientation.y)+"," 
                    + std::to_string(command.orientation.z) 
                    + "]), 0, 0, 4, 0.1, 300)"; 
         script_command_pub_->publish(msg);
         rclcpp::sleep_for(std::chrono::seconds(3));
     }
 
     /**
      * @brief Uses servo commands to move the UR3 to given joint angles.
      * @param command A collection of joint angles in radians.
      */
     void execute_command_joints_servo(CommandJoints command) {
         std_msgs::msg::String msg;
         msg.data = "servoj([" + std::to_string(command.joint_angles[0]) +"," 
                    + std::to_string(command.joint_angles[1])+"," 
                    + std::to_string(command.joint_angles[2])+"," 
                    + std::to_string(command.joint_angles[3])+"," 
                    + std::to_string(command.joint_angles[4])+"," 
                    + std::to_string(command.joint_angles[5]) 
                    + "], 0, 0, 6, 0.1, 300)"; 
         script_command_pub_->publish(msg);
         rclcpp::sleep_for(std::chrono::seconds(6));
     }
 
 private:
     rclcpp::Node::SharedPtr node_;
     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr script_command_pub_;
 };
 
 /**
  * @class ActionManager
  * Decides when to pick, place, or move the UR3 based on sensor data (e.g., from hand tracking).
  */
 class ActionManager : public rclcpp::Node
 {
 public:
     /**
      * @brief Constructor: sets up subscribers, commands, and the initial state.
      */
     ActionManager()
       : Node("ActionManager"),
         ur3_controller_(new UR3Controller(this)),
         command_key(" "),
         offset(0.047),
         object_in_gripper(false),
         picked_(false),
         placed_(false),
         approach(false)
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
          
         // Periodic evaluation of conditions
         timer_ = this->create_wall_timer(
             std::chrono::milliseconds(3000), 
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
 
     // Subscribers
     rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hand_state_sub_;
     rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hand_normal_sub_;
     rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr robot_position_sub_;
     rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hand_time_sub_;
     rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr hand_rate_of_change_sub_;
     rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
     rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hand_id_sub_;
 
     /**
      * @brief Creates the necessary ROS subscribers.
      */
     void setup_subscribers() {
         hand_state_sub_ = this->create_subscription<std_msgs::msg::Float32>(
             "hand_state", 1, 
             std::bind(&ActionManager::handStateCallback, this, std::placeholders::_1));
 
         hand_normal_sub_ = this->create_subscription<std_msgs::msg::Float32>(
             "hand_normal", 1, 
             std::bind(&ActionManager::handNormalCallback, this, std::placeholders::_1));
 
         robot_position_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
             "robot_position", 1, 
             std::bind(&ActionManager::handPositionCallback, this, std::placeholders::_1));
 
         hand_time_sub_ = this->create_subscription<std_msgs::msg::Float32>(
             "hand_time", 1, 
             std::bind(&ActionManager::handTimeCallback, this, std::placeholders::_1));
 
         hand_rate_of_change_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
             "hand_rate_of_change", 1, 
             std::bind(&ActionManager::handRateOfChangeCallback, this, std::placeholders::_1));
 
         joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
             "joint_states", 1, 
             std::bind(&ActionManager::jointStatesCallback, this, std::placeholders::_1));
 
         hand_id_sub_ = this->create_subscription<std_msgs::msg::Float32>(
             "hand_id", 1, 
             std::bind(&ActionManager::handidCallback, this, std::placeholders::_1));
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
 
     /**
      * @brief Sets up a map of command keys (e.g., "leftapproach") to joint angles.
      */
     void initialize_commands() {
         command_map_["leftapproach"] = {-1.588, -1.25, -0.98, -1.04, 1.58, 0};
         command_map_["lefttop"]      = {-1.66, -1.08, -1.48, -0.91, 1.61, 0};
         command_map_["leftcenter"]   = {-1.66, -1.08, -1.48, -0.91, 1.61, 0};
         command_map_["leftbottom"]   = {-1.71, -1.23, -1.72, -0.91, 1.61, 0};
         command_map_["topangles"]    = {1.371, -1.332, 1.011};
         command_map_["centerangles"] = {1.660, -1.720, 0.719};
         command_map_["bottomangles"] = {1.760, -1.541, 0.800};
         command_map_["rightapproach"] = {1.56, -2.04, 0.934, -1.73, -1.56, 0};
         command_map_["righttop"]     = {1.74, -2.012, 1.46, -2.19, -1.5, 0};
         command_map_["rightcenter"]  = {1.54, -2.23, 1.67, -1.83, -1.5, 0};
         command_map_["rightbottom"]  = {1.54, -1.94, 1.74, -2.19, -1.5, 0};
         command_map_["home"]         = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0};
     }
 
     /**
      * @brief Simple smoothing function for a Point.
      */
     geometry_msgs::msg::Point smooth_position(const geometry_msgs::msg::Point& new_pos, 
                                               const geometry_msgs::msg::Point& old_pos, 
                                               double alpha = 0.2)
     {
         geometry_msgs::msg::Point smoothed;
         smoothed.x = old_pos.x + alpha * (new_pos.x - old_pos.x);
         smoothed.y = old_pos.y + alpha * (new_pos.y - old_pos.y);
         smoothed.z = old_pos.z + alpha * (new_pos.z - old_pos.z);
         return smoothed;
     }
 
     /**
      * @brief Checks if a key is pressed in the terminal (non-blocking).
      * @return 1 if pressed, 0 otherwise.
      */
     int kbhit() {
         struct termios oldt, newt;
         int ch;
         int oldf;
 
         tcgetattr(STDIN_FILENO, &oldt);
         newt = oldt;
         newt.c_lflag &= ~(ICANON | ECHO);
         tcsetattr(STDIN_FILENO, TCSANOW, &newt);
         oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
         fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
         ch = getchar();
 
         tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
         fcntl(STDIN_FILENO, F_SETFL, oldf);
 
         if(ch != EOF) {
             ungetc(ch, stdin);
             return 1;
         }
         return 0;
     }
 
     /**
      * @brief Periodic function called by the timer to check and act upon conditions.
      */
     void evaluate_conditions_and_act() {
         // Check 'Enter' press for pick or drop
         if (kbhit()) {
             char c = getchar();
             if (c == '\n' && !object_in_gripper) {
                 ur3_controller_->pick_object_from_table();
                 object_in_gripper = true;
                 picked_ = true;
                 return;
             }
             else if(c == '\n' && object_in_gripper) {
                 ur3_controller_->drop_object_to_table();
                 object_in_gripper = false;
                 picked_ = false;
                 return;
             }
         }
 
         // Move to a certain position if a hand is in the correct state
         if (correct_hand_position() && picked_ && object_in_gripper) {
             bool condition = (should_approach_human() 
                               && (joint_states.position[0] > -1.59 
                                   && joint_states.position[0] < -1.56 
                                   && hand_time_ > 1))
                              || ((robot_position_.x > old_robot_position_.x 
                                   && robot_position_.x > 0 
                                   && last_location == "left")
                                  || (robot_position_.x < old_robot_position_.x 
                                      && robot_position_.x < 0 
                                      && last_location == "right"));
             if (condition) {
                 if((robot_position_.x > old_robot_position_.x && robot_position_.x > 0 && last_location == "left") 
                    || (robot_position_.x < old_robot_position_.x && robot_position_.x < 0 && last_location == "right"))
                 {
                     execute_command_joints(command_key_vec[1]);
                 }
                 execute_command_joints(command_key_vec[2]);
                 old_robot_position_ = robot_position_;
                 last_location = (robot_position_.x > 0) ? "right" : "left";
             }
         }
 
         // Place object if conditions are met
         if(should_place_object_operator() && correct_hand_position() && picked_ && object_in_gripper) {
             execute_command(command_key, offset);
 
             if(hand_rate_of_change_.x <= 8 && hand_rate_of_change_.x >= -8
                && hand_rate_of_change_.y <= 8 && hand_rate_of_change_.y >= -8
                && hand_rate_of_change_.z <= 8 && hand_rate_of_change_.z >= -8)
             {
                 rclcpp::sleep_for(std::chrono::seconds(2));
                 gripper.setGripperPosition(0x00);
                 rclcpp::sleep_for(std::chrono::seconds(1));
                 placed_ = true;
                 object_in_gripper = false;
             } else {
                 return;
             }
         }
 
         // Return to home if object is placed
         if (picked_ && placed_) {
             picked_ = false;
             placed_ = false;
             ur3_controller_->execute_home_pose();
             hand_id_ = old_hand_id;
         }
     }
 
     bool correct_hand_position() {
         return (hand_state_ >= 0 && hand_state_ <= 0.6) 
                && (hand_time_ > 2) 
                && (hand_normal_ > 0); 
     }
 
     bool should_approach_human() {
         command_key_vec[0] = "home";
         if(hand_normal_ > 0 && hand_state_ == 0 && object_in_gripper && !placed_ 
            && hand_time_ > 1 && robot_position_.x < 0)
         {
             location = "left";
             command_key_vec[1] = "leftapproach";
             if(robot_position_.z >= 0.31 && robot_position_.z < 0.51) {
                 command_key_vec[2] = "lefttop";
                 approach = true;
                 return true;
             }
             else if(robot_position_.z >= 0.21 && robot_position_.z < 0.31) {
                 command_key_vec[2] = "leftcenter";
                 approach = true;
                 return true;
             }
             else if(robot_position_.z >= 0.05 && robot_position_.z < 0.21) {
                 command_key_vec[2] = "leftbottom";
                 approach = true;
                 return true;
             } else {
                 return false;
             }
         }
         else if(hand_normal_ > 0 && hand_state_ == 0 && object_in_gripper && !placed_ 
                 && hand_time_ > 1 && robot_position_.x > 0)
         {
             location = "right";
             command_key_vec[1] = "rightapproach";
             if(robot_position_.z >= 0.31 && robot_position_.z < 0.51) {
                 command_key_vec[2] = "righttop";
                 approach = true;
                 return true;
             }
             else if(robot_position_.z >= 0.21 && robot_position_.z < 0.31) {
                 command_key_vec[2] = "rightcenter";
                 approach = true;
                 return true;
             }
             else if(robot_position_.z >= 0.05 && robot_position_.z < 0.21) {
                 command_key_vec[2] = "rightbottom";
                 approach = true;
                 return true;
             } else {
                 return false;
             }
         }
         return false; 
     }
 
     bool should_place_object_operator() {
         if(hand_normal_ > 0 && hand_state_ == 0 && object_in_gripper && (hand_time_ > 3) 
            && approach && ((location == "left" && robot_position_.x < 0) 
                           || (location == "right" && robot_position_.x > 0))
            && (hand_rate_of_change_.x <= 18 && hand_rate_of_change_.x >= -18
                && hand_rate_of_change_.y <= 18 && hand_rate_of_change_.y >= -18
                && hand_rate_of_change_.z <= 25 && hand_rate_of_change_.z >= -25))
         {
             if(robot_position_.z >= 0.31 && robot_position_.z < 0.51) {
                 command_key = "topangles";
                 return true;
             }
             else if(robot_position_.z >= 0.21 && robot_position_.z < 0.31) {
                 command_key = "centerangles";
                 return true;
             }
             else if(robot_position_.z >= 0.05 && robot_position_.z < 0.21) {
                 command_key = "bottomangles";
                 return true;
             }
         }
         approach = false;
         return false;
     }
 
     /**
      * @brief Executes a servo command for Cartesian movement, using a command key lookup.
      * @param command_key The lookup key in the command_map_.
      * @param offset A small Z-offset for the approach.
      */
     void execute_command(std::string command_key, float offset) {
         auto command_it = command_map_.find(command_key);
         if (command_it != command_map_.end()) {
             std::vector<double> joint_angles = command_it->second;
 
             Command command_joints;
             command_joints.orientation.x = joint_angles[0];
             command_joints.orientation.y = joint_angles[1];
             command_joints.orientation.z = joint_angles[2];
             
             command_joints.position.x = robot_position_.x;
             command_joints.position.y = robot_position_.y;
             command_joints.position.z = robot_position_.z + offset;
 
             ur3_controller_->execute_command_servo(command_joints);
         } else {
             RCLCPP_WARN(this->get_logger(), "Command not found: %s", command_key.c_str());
         }
     }
 
     /**
      * @brief Executes a servo command for joint movement, using a command key lookup.
      * @param command_key The lookup key in the command_map_.
      */
     void execute_command_joints(std::string command_key) {
         auto command_it = command_map_.find(command_key);
         if (command_it != command_map_.end()) {
             CommandJoints command_joints;
             command_joints.joint_angles = command_it->second;
             ur3_controller_->execute_command_joints_servo(command_joints);
         } else {
             RCLCPP_WARN(this->get_logger(), "Command not found: %s", command_key.c_str());
         }
     }
 };
 
 /**
  * @brief Main function: starts the ROS 2 node and spins the ActionManager.
  */
 int main(int argc, char *argv[])
 {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<ActionManager>());
     rclcpp::shutdown();
     return 0;
 }
 