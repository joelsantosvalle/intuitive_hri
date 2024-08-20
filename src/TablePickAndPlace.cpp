#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class TablePickAndPlace : public rclcpp::Node
{
public:
    TablePickAndPlace()
        : Node("table_pick_and_place"), pickup_(true), dropoff_(false)
    {
        // Publisher for joint trajectory
        pub_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/scaled_joint_trajectory_controller/joint_trajectory", 10);

        // Timer to trigger motion sequence
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TablePickAndPlace::execute_motion, this));
    }

private:
    // Function to create a JointTrajectoryPoint
    trajectory_msgs::msg::JointTrajectoryPoint create_trajectory_point(
        std::vector<double> positions, double time_from_start_sec, double velocity = 0.1, double acceleration = 0.05)
    {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = positions;
        point.velocities.resize(positions.size(), velocity);
        point.accelerations.resize(positions.size(), acceleration);
        point.time_from_start.sec = static_cast<int32_t>(time_from_start_sec);
        point.time_from_start.nanosec = 0;
        return point;
    }

    // Function to execute the motion sequence
    void execute_motion()
    {
        trajectory_msgs::msg::JointTrajectory trajectory_msg;
        trajectory_msg.header.stamp = this->now();
        trajectory_msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

        if (pickup_)
        {
            // Step 1: Turn around to pick object
            trajectory_msg.points.push_back(create_trajectory_point({-1.57, 0.0, 0.0, 0.0, 0.0, 0.0}, 1.0));

            // Step 2: Approach object
            trajectory_msg.points.push_back(create_trajectory_point({-1.57, -0.3, 0.0, 0.0, 0.0, 0.0}, 2.0));

            // Step 3: Close gripper (Assume gripper control is not included in this example)
            // (Gripper control code would go here if needed)

            // Step 4: Lift back up
            trajectory_msg.points.push_back(create_trajectory_point({-1.57, 0.0, 0.0, 0.0, 0.0, 0.0}, 3.0));

            // Step 5: Turn back around
            trajectory_msg.points.push_back(create_trajectory_point({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 4.0));

            // Now switch to dropoff mode
            pickup_ = false;
            dropoff_ = true;
        }
        else if (dropoff_)
        {
            // Step 6: Turn to drop off position
            trajectory_msg.points.push_back(create_trajectory_point({-1.57, 0.0, 0.0, 0.0, 0.0, 0.0}, 1.0));

            // Step 7: Lower the object
            trajectory_msg.points.push_back(create_trajectory_point({-1.57, -0.3, 0.0, 0.0, 0.0, 0.0}, 2.0));

            // Step 8: Open gripper (Assume gripper control is not included in this example)
            // (Gripper control code would go here if needed)

            // Step 9: Lift back up
            trajectory_msg.points.push_back(create_trajectory_point({-1.57, 0.0, 0.0, 0.0, 0.0, 0.0}, 3.0));

            // Step 10: Return to initial position
            trajectory_msg.points.push_back(create_trajectory_point({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 4.0));

            // Reset for next cycle
            dropoff_ = false;
            pickup_ = true;
        }

        // Publish the trajectory message
        pub_trajectory_->publish(trajectory_msg);
    }

    // Publisher
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_trajectory_;

    // Timer to trigger the motion sequence
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables to track the robot's state
    bool pickup_;
    bool dropoff_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TablePickAndPlace>());
    rclcpp::shutdown();
    return 0;
}
