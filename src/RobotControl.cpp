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

// Sensor data structure
struct SensorData {
    float hand_state;
    float hand_normal;
    geometry_msgs::msg::Point hand_position;
    float hand_time_in_sensor;
    geometry_msgs::msg::Point hand_rate_of_change;
};

//Class for communication with the UR3

class UR3Controller {
public:
    UR3Controller(rclcpp::Node* node)
        : node_(node) {
        script_command_pub_ = node_->create_publisher<std_msgs::msg::String>("/urscript_interface/script_command", 10);
    }

    void execute_command(std::vector<double> points) {
        std_msgs::msg::String msg;
        //msg.data = " movej(["  + std::to_string(-1.54) +"," +std::to_string(-1.57)+"," +std::to_string(0)+"," +std::to_string(-1.57)+"," +std::to_string(0)+","+ std::to_string(0) + "], a=1.2, v=0.32, r=0.01)"; 
        //script_command_pub_->publish(msg);
        //rclcpp::sleep_for(std::chrono::seconds(5));
        //msg.data = " movej(["  + std::to_string(-1.55) +"," +std::to_string(-1.52)+"," +std::to_string(1.51)+"," +std::to_string(-1.487)+"," +std::to_string(0)+","+ std::to_string(0) + "], a=1.2, v=0.32, r=0.01)"; 
        //script_command_pub_->publish(msg);
        //rclcpp::sleep_for(std::chrono::seconds(5));
        //msg.data = " movej(["  + std::to_string(-1.51) +"," +std::to_string(-1.54)+"," +std::to_string(1.51)+"," +std::to_string(-1.6)+"," +std::to_string(-1.45)+","+ std::to_string(6.26) + "], a=1.2, v=0.32, r=0.01)"; 
        //t_command_pub_->publish(msg);
        //rclcpp::sleep_for(std::chrono::seconds(4.3));
        msg.data = " movej(["  + std::to_string(-1.07) +"," +std::to_string(-0.747)+"," +std::to_string(0.67)+"," +std::to_string(-1.56)+"," +std::to_string(-1.51)+","+ std::to_string(0.62) + "], a=1.2, v=0.25, r=0.01)"; 
        script_command_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(6));
        msg.data = " movej(["  + std::to_string(-1.0492) +"," +std::to_string(-0.586)+"," +std::to_string(0.685)+"," +std::to_string(-1.614)+"," +std::to_string(-1.506)+","+ std::to_string(0.495) + "], a=1.0, v=0.15, r=0.01)"; 
        script_command_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(8));
        //msg.data = " movej(["  + std::to_string(-1.07) +"," +std::to_string(-0.747)+"," +std::to_string(0.67)+"," +std::to_string(-1.56)+"," +std::to_string(-1.51)+","+ std::to_string(0.62) + "], a=1.2, v=0.25, r=0.01)"; 
        //script_command_pub_->publish(msg);
        //rclcpp::sleep_for(std::chrono::seconds(1));
        //msg.data = " movej(["  + std::to_string(-1.51) +"," +std::to_string(-1.54)+"," +std::to_string(1.51)+"," +std::to_string(-1.6)+"," +std::to_string(-1.45)+","+ std::to_string(6.26) + "], a=1.2, v=0.32, r=0.01)"; 
        //script_command_pub_->publish(msg);
        //msg.data = " movej(["  + std::to_string(-1.55) +"," +std::to_string(-1.52)+"," +std::to_string(1.51)+"," +std::to_string(-1.487)+"," +std::to_string(0)+","+ std::to_string(0) + "], a=1.2, v=0.32, r=0.01)"; 
        //script_command_pub_->publish(msg);
        //msg.data = " movej(["  + std::to_string(-1.54) +"," +std::to_string(-1.57)+"," +std::to_string(0)+"," +std::to_string(-1.57)+"," +std::to_string(0)+","+ std::to_string(0) + "], a=1.2, v=0.32, r=0.01)"; 
        //script_command_pub_->publish(msg);
        msg.data = " movej(["  + std::to_string(0) +"," +std::to_string(-1.57)+"," +std::to_string(0)+"," +std::to_string(-1.57)+"," +std::to_string(0)+","+ std::to_string(0) + "], a=1.2, v=0.32, r=0.01)"; 
        script_command_pub_->publish(msg);
    }

    void execute_command(Command command) {
        std::ostringstream script;
    
        script << "def my_prog():\n"
               << "  movej(p[" << command.position.x << ", " << command.position.y << ", " 
               << command.position.z << ", " << command.orientation.x << ", " 
               << command.orientation.y << ", " << command.orientation.z << "], a=1.2, v=0.25, r=0)\n"
               << "  textmsg(\"motion finished\")\n"
               << "end";

        std_msgs::msg::String msg;
        msg.data = script.str();
        script_command_pub_->publish(msg);
    }

    void execute_command_joints(CommandJoints command) {
        std::ostringstream script;
        script << "def my_prog():\n"
               << "  movej([" << command.joint_angles[0] << ", " << command.joint_angles[1] << ", " 
               << command.joint_angles[2] << ", " << command.joint_angles[3] << ", " 
               << command.joint_angles[4] << ", " << command.joint_angles[5] << "], a=1.2, v=0.25, r=0)\n"
               << "  textmsg(\"motion finished\")\n"
               << "end";

        std_msgs::msg::String msg;
        msg.data = script.str();
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
    ActionManager() : Node("ActionManager"),ur3_controller_(new UR3Controller(this)) ,command_key(" ")
    {
        std::cout << "hello 1" <<std::endl;
        setup_subscribers();
        std::cout << "hello 1" <<std::endl;
        initialize_commands();
        std::cout << "hello 1" <<std::endl;
        
        std::cout << "hello 1" <<std::endl;
        int one = 1;
        if(one == 1)
        {
            std::vector move = {0.070, -0.409, 0.654, 1.504, -0.349, 0.494};
            ur3_controller_->execute_command(move);
            one++;
        }
    }

private:
    SensorData sensor_data_;
    std::map<std::string, std::vector<double>> command_map_;
    std::shared_ptr<UR3Controller> ur3_controller_;
    std::string command_key;

    void setup_subscribers() {
        // Subscriber to hand state
        hand_state_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "hand_state", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                this->sensor_data_callback(msg, "hand_state");
            });

        // Subscriber to hand normal
        hand_normal_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "hand_normal", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                this->sensor_data_callback(msg, "hand_normal");
            });

        // Subscriber to hand position
        hand_position_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "hand_position_sensor", 10,
            [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                this->sensor_data_callback(msg, "hand_position");
            });

        // Subscriber to hand time
        hand_time_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "hand_time", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                this->sensor_data_callback(msg, "hand_time");
            });

        // Subscriber to hand rate of change
        hand_rate_of_change_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "hand_rate_of_change", 10,
            [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                this->sensor_data_callback(msg, "hand_rate_of_change");
            });
    }

    void sensor_data_callback(const std_msgs::msg::Float32::SharedPtr msg, const std::string& topic) {
        // Handle Float32 messages
        RCLCPP_INFO(this->get_logger(), "Received %s: %f", topic.c_str(), msg->data);

        
    }

        void sensor_data_callback(const geometry_msgs::msg::Point::SharedPtr msg, const std::string& topic) {
        // Handle Point messages
        RCLCPP_INFO(this->get_logger(), "Received %s: x: %f, y: %f, z: %f", topic.c_str(), msg->x, msg->y, msg->z);
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
        command_map_["center"] = {0.5, -1.0, 1.0};
        command_map_["centertop"] = {0.0, -1.57, 1.0};
        command_map_["centerleftback"] = {0.5, -1.0, 1.5};
        command_map_["centerrightback"] = {0.0, -1.57, 1.0};
        command_map_["home"] = {0.0, -1.57, 0.5, 0.0, 0.0, 0.0};
    }

    void update_sensor_data(const std_msgs::msg::Float32::SharedPtr& hand_state,
                            const std_msgs::msg::Float32::SharedPtr& hand_normal,
                            const geometry_msgs::msg::Point::SharedPtr& hand_position,
                            const std_msgs::msg::Float32::SharedPtr& hand_time,
                            const geometry_msgs::msg::Point::SharedPtr& hand_rate_of_change)
    {
        sensor_data_.hand_state = hand_state->data;
        sensor_data_.hand_normal = hand_normal->data;
        sensor_data_.hand_position = *hand_position;
        sensor_data_.hand_time_in_sensor = hand_time->data;
        sensor_data_.hand_rate_of_change = *hand_rate_of_change;
    }

    void evaluate_conditions_and_act() {
        if (should_pick_object_table()) {
            execute_command_joints(command_key); 
        } else if (should_place_object_table()) {
            execute_command_joints(command_key);
        } else if (should_approach_human() && correct_hand_position()) {
            execute_command(command_key);
        } else if (should_place_object_operator() && correct_hand_position()) {
            execute_command(command_key);
        } else if (should_change_position() && correct_hand_position()) {
            execute_command(command_key);
        } else if (should_turn() && correct_hand_position()) {
            execute_command_joints(command_key);
        } else {
            maintain_current_state();
        }
    }

    void execute_command(std::string command_key) {
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
            command_joints.position.x = sensor_data_.hand_position.x;
            command_joints.position.y = sensor_data_.hand_position.y;
            command_joints.position.z = sensor_data_.hand_position.z;
            
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
        return true;
    }

    bool should_pick_object_table() {
        return true; //sensor_data_.hand_state > 0.5 && sensor_data_.hand_time_in_sensor > 2.0;
    }

    bool should_place_object_table() {
        // Example condition for placing an object
        return true; //sensor_data_.hand_normal > 0.7 && sensor_data_.hand_rate_of_change.z > 0.3;
    }

    bool should_approach_human() {
        // Example condition for approaching a human
        return true; //sensor_data_.hand_position.x < 0.5 && sensor_data_.hand_position.y > 1.0;
    }

    bool should_place_object_operator(){
        // Example condition for approaching a human
        return true; //sensor_data_.hand_position.x < 0.5 && sensor_data_.hand_position.y > 1.0;
    }

    bool should_change_position(){
        // Example condition for approaching a human
        return true; //sensor_data_.hand_position.x < 0.5 && sensor_data_.hand_position.y > 1.0;
    }

    bool should_turn(){
        // Example condition for approaching a human
        return true; //sensor_data_.hand_position.x < 0.5 && sensor_data_.hand_position.y > 1.0;
    }

    void maintain_current_state() {
        RCLCPP_INFO(this->get_logger(), "Maintaining current state");
    }

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hand_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hand_normal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr hand_position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hand_time_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr hand_rate_of_change_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::cout << "hello 1" <<std::endl;
    rclcpp::spin(std::make_shared<ActionManager>());
    std::cout << "hello 1" <<std::endl;
    rclcpp::shutdown();
    return 0;
}
