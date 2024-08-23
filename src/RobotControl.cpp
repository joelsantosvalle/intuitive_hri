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

// Command structure to hold position and joint angles
struct Command {
    geometry_msgs::msg::Vector3 position;  // Dynamic x, y, z values
    geometry_msgs::msg::Vector3 orientation;      // Predefined joint angles
};

// Command structure to hold position and joint angles
struct CommandJoints {
    geometry_msgs::msg::Vector3 position;  // Dynamic x, y, z values
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
    UR3Controller(rclcpp::Node::SharedPtr node)
        : node_(node) {
        script_command_pub_ = node_->create_publisher<std_msgs::msg::String>("/urscript_interface/script_command", 10);
    }

    void execute_command(const Command& command) {
        std::ostringstream script;
        // URScript generation based on predefined joint angles and dynamic x, y, z values
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

    void execute_command_joints(const Command& command) {
        std::ostringstream script;
        // URScript generation based on predefined joint angles and dynamic x, y, z values
        script << "def my_prog():\n"
               << "  movej([" << command.position.x << ", " << command.position.y << ", " 
               << command.position.z << ", " << command.orientation.x << ", " 
               << command.orientation.y << ", " << command.orientation.z << "], a=1.2, v=0.25, r=0)\n"
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

class CommandSelector : public rclcpp::Node
{
public:
    CommandSelector() 
    : Node("command_selector"), 
      ur3_controller_(std::make_shared<UR3Controller>(this->shared_from_this())) 
    {
        setup_subscribers();
        setup_synchronizer();
        initialize_commands();
    }

private:
    SensorData sensor_data_;
    std::map<std::string, Command> command_map_;
    std::shared_ptr<UR3Controller> ur3_controller_;
    
    // Subscribers
    message_filters::Subscriber<std_msgs::msg::Float32> hand_state_sub_;
    message_filters::Subscriber<std_msgs::msg::Float32> hand_normal_sub_;
    message_filters::Subscriber<geometry_msgs::msg::Point> hand_position_sub_;
    message_filters::Subscriber<std_msgs::msg::Float32> hand_time_sub_;
    message_filters::Subscriber<geometry_msgs::msg::Point> hand_rate_of_change_sub_;

    // Synchronizer
    std::shared_ptr<message_filters::TimeSynchronizer<
        std_msgs::msg::Float32, std_msgs::msg::Float32, geometry_msgs::msg::Point, 
        std_msgs::msg::Float32, geometry_msgs::msg::Point>> sync_;

    void setup_subscribers() {
        hand_state_sub_.subscribe(this, "hand_state");
        hand_normal_sub_.subscribe(this, "hand_normal");
        hand_position_sub_.subscribe(this, "hand_position_sensor");
        hand_time_sub_.subscribe(this, "hand_time");
        hand_rate_of_change_sub_.subscribe(this, "hand_rate_of_change");
    }

    void setup_synchronizer() {
        sync_.reset(new message_filters::TimeSynchronizer<
            std_msgs::msg::Float32, std_msgs::msg::Float32, geometry_msgs::msg::Point, 
            std_msgs::msg::Float32, geometry_msgs::msg::Point>(
            hand_state_sub_, hand_normal_sub_, hand_position_sub_, hand_time_sub_, 
            hand_rate_of_change_sub_, 10));

        sync_->registerCallback(std::bind(&CommandSelector::sensor_data_callback, this, 
                                          std::placeholders::_1, std::placeholders::_2, 
                                          std::placeholders::_3, std::placeholders::_4, 
                                          std::placeholders::_5));
    }

    void initialize_commands() {
        command_map_["left"] = Command{geometry_msgs::msg::Vector3{1.0, 0.0, 0.0}};
        command_map_["lefttop"] = Command{geometry_msgs::msg::Vector3{0.0, 1.0, 0.0}};
        command_map_["leftcenter"] = Command{geometry_msgs::msg::Vector3{0.0, 0.0, 1.0}};
        command_map_["leftbottom"] = Command{geometry_msgs::msg::Vector3{1.0, 0.0, 0.0}};
        command_map_["right"] = Command{geometry_msgs::msg::Vector3{0.0, 1.0, 0.0}};
        command_map_["righttop"] = Command{geometry_msgs::msg::Vector3{0.0, 0.0, 1.0}};
        command_map_["rightcenter"] = Command{geometry_msgs::msg::Vector3{1.0, 0.0, 0.0}};
        command_map_["rightbottom"] = Command{geometry_msgs::msg::Vector3{0.0, 1.0, 0.0}};
        command_map_["command9"] = Command{geometry_msgs::msg::Vector3{0.0, 0.0, 1.0}};
        command_map_["command10"] = Command{geometry_msgs::msg::Vector3{1.0, 0.0, 0.0}};
        command_map_["command11"] = Command{geometry_msgs::msg::Vector3{0.0, 1.0, 0.0}};
        command_map_["command12"] = Command{geometry_msgs::msg::Vector3{0.0, 0.0, 1.0}};
    }

    void sensor_data_callback(const std_msgs::msg::Float32::SharedPtr hand_state,
                              const std_msgs::msg::Float32::SharedPtr hand_normal,
                              const geometry_msgs::msg::Point::SharedPtr hand_position,
                              const std_msgs::msg::Float32::SharedPtr hand_time,
                              const geometry_msgs::msg::Point::SharedPtr hand_rate_of_change)
    {
        update_sensor_data(hand_state, hand_normal, hand_position, hand_time, hand_rate_of_change);
        evaluate_conditions_and_act();
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
        if (should_pick_object()) {
            execute_command("command1");  // Example command
        } else if (should_place_object()) {
            execute_command("command2");
        } else if (should_approach_human()) {
            execute_command("command3");
        } else {
            maintain_current_state();
        }
    }

    bool should_pick_object() {
        // Example condition for picking an object
        return sensor_data_.hand_state > 0.5 && sensor_data_.hand_time_in_sensor > 2.0;
    }

    bool should_place_object() {
        // Example condition for placing an object
        return sensor_data_.hand_normal > 0.7 && sensor_data_.hand_rate_of_change.z > 0.3;
    }

    bool should_approach_human() {
        // Example condition for approaching a human
        return sensor_data_.hand_position.x < 0.5 && sensor_data_.hand_position.y > 1.0;
    }

    void execute_command(const std::string& command_key) {
        auto command_it = command_map_.find(command_key);
        if (command_it != command_map_.end()) {
            Command command = command_it->second;
            // Updating the dynamic position from sensor data
            command.position.x = sensor_data_.hand_position.x;
            command.position.y = sensor_data_.hand_position.y;
            command.position.z = sensor_data_.hand_position.z;
            ur3_controller_->execute_command(command);
        }
    }

    void maintain_current_state() {
        RCLCPP_INFO(this->get_logger(), "Maintaining current state");
        // Add any logic to maintain the current state if no action is needed
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandSelector>());
    rclcpp::shutdown();
    return 0;
}
