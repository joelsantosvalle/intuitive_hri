/**
 * LeapMotionPublisher.cpp
 *
 * @date 17.06.2024
 * @author Joel Santos
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <array>

using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @class LeapDataNode
 * @brief A ROS 2 node that receives Leap Motion data via UDP and publishes it to various topics.
 *
 * This node binds a UDP socket to a predefined port (57410) on localhost. It periodically checks
 * for incoming float data (21 floats) and publishes them as ROS 2 messages on several topics:
 *  - number_of_hands (std_msgs::msg::Float32)
 *  - hand_state (std_msgs::msg::Float32)
 *  - hand_id (std_msgs::msg::Float32)
 *  - hand_normal (std_msgs::msg::Float32)
 *  - hand_position_sensor (geometry_msgs::msg::Point)
 *  - hand_time (std_msgs::msg::Float32)
 *  - hand_orientation_sensor (geometry_msgs::msg::Quaternion)
 *  - hand_rate_of_change (geometry_msgs::msg::Point)
 */

class LeapDataNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor. Initializes publishers, sets up the UDP socket, and starts a timer.
     */
    LeapDataNode() : Node("leap_data")
    {
        // Create publishers for different types of Leap data
        pub_number_of_hands_ = this->create_publisher<std_msgs::msg::Float32>("hand_number", 1);
        pub_hand_state_ = this->create_publisher<std_msgs::msg::Float32>("hand_state", 1);
        pub_hand_id_ = this->create_publisher<std_msgs::msg::Float32>("hand_id", 1);
        pub_hand_normal_ = this->create_publisher<std_msgs::msg::Float32>("hand_normal", 1);
        pub_palm_position_stable_ = this->create_publisher<geometry_msgs::msg::Point>("hand_position_sensor", 1);
        pub_hand_time_in_sensor_ = this->create_publisher<std_msgs::msg::Float32>("hand_time", 1);
        pub_hand_orientation_ = this->create_publisher<geometry_msgs::msg::Quaternion>("hand_orientation_sensor", 1);
        pub_hand_rate_of_change_ = this->create_publisher<geometry_msgs::msg::Point>("hand_rate_of_change", 1);

        // Create a UDP socket
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) {
            perror("socket creation failed");
            exit(EXIT_FAILURE);
        }

        // Set up the server address struct
        struct sockaddr_in servaddr;
        memset(&servaddr, 0, sizeof(servaddr));

        // Configure the socket for localhost (127.0.0.1) on port 57410
        servaddr.sin_family = AF_INET; 
        servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
        servaddr.sin_port = htons(57410);

        // Bind the socket to this address
        if (bind(udp_socket_, reinterpret_cast<const struct sockaddr *>(&servaddr), sizeof(servaddr)) < 0) {
            perror("bind failed");
            close(udp_socket_);
            exit(EXIT_FAILURE);
        }

        // Create a timer that triggers 'timer_callback' every 1 ms
        // This function will check for incoming data
        timer_ = this->create_wall_timer(
            1ms, std::bind(&LeapDataNode::timer_callback, this));
    }

private:
    /**
     * @brief Callback function triggered by the timer.
     *
     * Attempts to receive up to 21 floats of Leap Motion data from the UDP socket.
     * Upon successful reception, it publishes each relevant portion of the data
     * to its corresponding topic.
     */
    void timer_callback()
    {
        // We'll receive up to 21 floats from the Leap Motion sender
        std::array<float, 21> data = {};

        // We'll store client address info here (although unused except for recvfrom)
        struct sockaddr_in cliaddr;
        socklen_t len = sizeof(cliaddr);

        // Receive the data; block until data arrives or an error occurs
        ssize_t n = recvfrom(udp_socket_, data.data(), data.size() * sizeof(float), MSG_WAITALL,
                             reinterpret_cast<struct sockaddr *>(&cliaddr), &len);
        if (n < 0) {
            perror("recvfrom failed");
            return;
        }

        // 1) Number of hands in the current frame
        auto number_of_hand_in_frame = std_msgs::msg::Float32();
        number_of_hand_in_frame.data = data[0];
        pub_number_of_hands_->publish(number_of_hand_in_frame);

        // 2) Hand state or "grab strength" data
        auto strength = std_msgs::msg::Float32();
        strength.data = data[1];
        pub_hand_state_->publish(strength);

        // 3) The ID of the currently detected hand
        auto hand_identifier = std_msgs::msg::Float32();
        hand_identifier.data = data[2];
        pub_hand_id_->publish(hand_identifier);

        // 4) The palm's position (X, Y, Z) converted from millimeters to meters
        auto coordinates = geometry_msgs::msg::Point();
        coordinates.x = data[3] * 0.001;
        coordinates.y = data[4] * 0.001;
        coordinates.z = data[5] * 0.001;
        pub_palm_position_stable_->publish(coordinates);

        // 5) How long the hand has been visible (in seconds, from microseconds)
        auto hand_time_in_sensor = std_msgs::msg::Float32();
        hand_time_in_sensor.data = (data[6] / 1000000.0f); 
        pub_hand_time_in_sensor_->publish(hand_time_in_sensor);

        // 6) The palm's normal direction (Y-component)
        auto palm_direction = std_msgs::msg::Float32();
        palm_direction.data = data[7];
        pub_hand_normal_->publish(palm_direction);
        
        // 7) Rate of change or velocity (X, Y, Z)
        auto rate_of_change = geometry_msgs::msg::Point();
        rate_of_change.x = data[8];
        rate_of_change.y = data[9];
        rate_of_change.z = data[10];
        pub_hand_rate_of_change_->publish(rate_of_change);

        // 8) Quaternion representing the hand's orientation (X, Y, Z, W)
        auto orientation_of_hand = geometry_msgs::msg::Quaternion();
        orientation_of_hand.x = data[11];
        orientation_of_hand.y = data[12];
        orientation_of_hand.z = data[13];
        orientation_of_hand.w = data[14];
        pub_hand_orientation_->publish(orientation_of_hand);
    }

    // Publishers for each piece of Leap data
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_number_of_hands_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_hand_state_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_hand_id_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_hand_normal_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_palm_position_stable_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_hand_time_in_sensor_;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr pub_hand_orientation_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_hand_rate_of_change_;

    // UDP socket and timer
    int udp_socket_;
    rclcpp::TimerBase::SharedPtr timer_;
};

/**
 * @brief Main entry point of the ROS 2 node.
 *
 * Initializes the rclcpp library, instantiates LeapDataNode,
 * and spins until shutdown.
 */
int main(int argc, char *argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create and run a single-node executor for LeapDataNode
    rclcpp::spin(std::make_shared<LeapDataNode>());

    // Shutdown ROS 2 once the node is done
    rclcpp::shutdown();
    return 0;
}
