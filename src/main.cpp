/*
#include "rclcpp/rclcpp.hpp"
#include "SensorDataNode.hpp"       // Node for retrieving sensor data
#include "DataPublisherNode.hpp"    // Node for publishing data
#include "DataConverterNode.hpp"    // Node for converting data
#include "RobotControlNode.hpp"     // Node for controlling the robot

int main(int argc, char *argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create instances of your node classes
    auto sensor_data_node = std::make_shared<SensorDataNode>();
    auto data_publisher_node = std::make_shared<DataPublisherNode>();
    auto data_converter_node = std::make_shared<DataConverterNode>();
    auto robot_control_node = std::make_shared<RobotControlNode>();

    // Main loop: retrieve, publish, convert, control
    while (rclcpp::ok()) {
        // 1. Retrieve data
        sensor_data_node->retrieve_data();

        // 2. Publish data
        data_publisher_node->publish_data(sensor_data_node->get_data());

        // 3. Convert data
        auto converted_data = data_converter_node->convert_data(sensor_data_node->get_data());

        // 4. Control robot
        robot_control_node->control_robot(converted_data);

        // Optionally, add a sleep to control loop rate
        rclcpp::Rate loop_rate(10);  // 10 Hz
        loop_rate.sleep();
    }

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
*/

