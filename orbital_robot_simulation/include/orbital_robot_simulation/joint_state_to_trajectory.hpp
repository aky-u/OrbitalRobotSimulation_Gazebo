#ifndef JOINT_STATE_TO_TRAJECTORY_NODE_HPP_
#define JOINT_STATE_TO_TRAJECTORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace orbital_robot_simulation
{
class JointStateToTrajectoryNode : public rclcpp::Node
{
public:
    JointStateToTrajectoryNode();

private:
    void jointStateCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    sensor_msgs::msg::JointState::SharedPtr joint_state_msg_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;    
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
};
} // namespace orbital_robot_simulation

#endif  // JOINT_STATE_TO_TRAJECTORY_NODE_HPP_
