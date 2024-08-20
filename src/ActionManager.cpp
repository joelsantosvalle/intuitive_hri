#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

class ActionManager : public rclcpp::Node
{
public:
    ActionManager()
        : Node("action_manager")
    {
        // Create boolean publishers
        pub_pickup_from_table_ = this->create_publisher<std_msgs::msg::Bool>("pickup_from_table", 10);
        pub_pickup_from_operator_ = this->create_publisher<std_msgs::msg::Bool>("pickup_from_operator", 10);
        pub_dropoff_to_table_ = this->create_publisher<std_msgs::msg::Bool>("dropoff_to_table", 10);
        pub_dropoff_to_operator_ = this->create_publisher<std_msgs::msg::Bool>("dropoff_to_operator", 10);

        // Create subscriptions to other topics
        sub_number_of_hands_ = this->create_subscription<std_msgs::msg::Float32>(
            "hand_number", 10, std::bind(&ActionManager::number_of_hands_callback, this, std::placeholders::_1));

        sub_hand_state_ = this->create_subscription<std_msgs::msg::Float32>(
            "hand_state", 10, std::bind(&ActionManager::hand_state_callback, this, std::placeholders::_1));

        sub_hand_id_ = this->create_subscription<std_msgs::msg::Float32>(
            "hand_id", 10, std::bind(&ActionManager::hand_id_callback, this, std::placeholders::_1));

        sub_hand_normal_ = this->create_subscription<std_msgs::msg::Float32>(
            "hand_normal", 10, std::bind(&ActionManager::hand_normal_callback, this, std::placeholders::_1));

        sub_palm_position_stable_ = this->create_subscription<geometry_msgs::msg::Point>(
            "hand_position_sensor", 10, std::bind(&ActionManager::palm_position_callback, this, std::placeholders::_1));

        sub_hand_time_in_sensor_ = this->create_subscription<std_msgs::msg::Float32>(
            "hand_time", 10, std::bind(&ActionManager::hand_time_in_sensor_callback, this, std::placeholders::_1));

        sub_hand_orientation_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "hand_orientation_sensor", 10, std::bind(&ActionManager::hand_orientation_callback, this, std::placeholders::_1));

        sub_hand_rate_of_change_ = this->create_subscription<geometry_msgs::msg::Point>(
            "hand_rate_of_change", 10, std::bind(&ActionManager::hand_rate_of_change_callback, this, std::placeholders::_1));
    }

private:
    // Boolean publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_pickup_from_table_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_pickup_from_operator_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_dropoff_to_table_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_dropoff_to_operator_;

    //function to publish whether true or false

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionManager>());
    rclcpp::shutdown();
    return 0;
}
