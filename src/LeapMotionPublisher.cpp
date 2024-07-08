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

class LeapDataNode : public rclcpp::Node
{
public:
    LeapDataNode() : Node("leap_data")
    {
        pub_number_of_hands_ = this->create_publisher<std_msgs::msg::Float32>("hand_number", 1);
        pub_hand_state_ = this->create_publisher<std_msgs::msg::Float32>("hand_state", 1);
        pub_hand_id_ = this->create_publisher<std_msgs::msg::Float32>("hand_id", 1);
        pub_hand_normal_ = this->create_publisher<std_msgs::msg::Float32>("hand_normal", 1);
        pub_palm_position_stable_ = this->create_publisher<geometry_msgs::msg::Point>("hand_position_sensor", 1);
        pub_life_of_hand_ = this->create_publisher<std_msgs::msg::Float32>("hand_time", 1);
        pub_hand_orientation_ = this->create_publisher<geometry_msgs::msg::Quaternion>("hand_orientation_sensor", 1);
        pub_hand_rate_of_change_ = this->create_publisher<geometry_msgs::msg::Point>("hand_rate_of_change", 1);

        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) {
            perror("socket creation failed");
            exit(EXIT_FAILURE);
        }

        struct sockaddr_in servaddr;
        memset(&servaddr, 0, sizeof(servaddr));

        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
        servaddr.sin_port = htons(57410);

        if (bind(udp_socket_, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
            perror("bind failed");
            close(udp_socket_);
            exit(EXIT_FAILURE);
        }

        timer_ = this->create_wall_timer(
            20ms, std::bind(&LeapDataNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        std::array<float, 21> data = {};
        struct sockaddr_in cliaddr;
        socklen_t len = sizeof(cliaddr);
        ssize_t n = recvfrom(udp_socket_, data.data(), data.size() * sizeof(float), MSG_WAITALL, (struct sockaddr *)&cliaddr, &len);
        if (n < 0) {
            perror("recvfrom failed");
            return;
        }

        auto number_of_hand_in_frame = std_msgs::msg::Float32();
        number_of_hand_in_frame.data = data[0];
        pub_number_of_hands_->publish(number_of_hand_in_frame);

        auto strength = std_msgs::msg::Float32();
        strength.data = data[1];
        pub_hand_state_->publish(strength);

        auto hand_identifier = std_msgs::msg::Float32();
        hand_identifier.data = data[2];
        pub_hand_id_->publish(hand_identifier);

        auto coordinates = geometry_msgs::msg::Point();
        coordinates.x = data[3] * 0.001;
        coordinates.y = data[4] * 0.001;
        coordinates.z = data[5] * 0.001;
        pub_palm_position_stable_->publish(coordinates);

        auto life_of_hand_in_sensor = std_msgs::msg::Float32();
        life_of_hand_in_sensor.data = data[6];
        pub_life_of_hand_->publish(life_of_hand_in_sensor);

        auto palm_direction = std_msgs::msg::Float32();
        palm_direction.data = data[7];
        pub_hand_normal_->publish(palm_direction);
        
        auto rate_of_change = geometry_msgs::msg::Point();
        rate_of_change.x = data[8];
        rate_of_change.y = data[9];
        rate_of_change.z = data[10];
        pub_hand_rate_of_change_->publish(rate_of_change);

        auto orientation_of_hand = geometry_msgs::msg::Quaternion();
        orientation_of_hand.x = data[11];
        orientation_of_hand.y = data[12];
        orientation_of_hand.z = data[13];
        orientation_of_hand.w = data[14];
        pub_hand_orientation_->publish(orientation_of_hand);
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_number_of_hands_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_hand_state_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_hand_id_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_hand_normal_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_palm_position_stable_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_life_of_hand_;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr pub_hand_orientation_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_hand_rate_of_change_;

    int udp_socket_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LeapDataNode>());
    rclcpp::shutdown();
    return 0;
}
