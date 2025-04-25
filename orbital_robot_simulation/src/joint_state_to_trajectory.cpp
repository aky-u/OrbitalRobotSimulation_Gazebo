#include "joint_state_to_trajectory.hpp"

namespace orbital_robot_simulation
{
JointStateToTrajectoryNode::JointStateToTrajectoryNode()
    : Node("joint_state_to_trajectory_node")
{
  joint_state_msg_ = std::make_shared<sensor_msgs::msg::JointState>();

  // Create JointState topic subscriber
  joint_state_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_state_command", 10, std::bind(&JointStateToTrajectoryNode::jointStateCommandCallback, this, std::placeholders::_1));
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&JointStateToTrajectoryNode::jointStateCallback, this, std::placeholders::_1));
  // Create JointTrajectory topic publisher
  joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/feedback_effort_controller/joint_trajectory", 10);
}

void JointStateToTrajectoryNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  joint_state_msg_ = msg;
}

void JointStateToTrajectoryNode::jointStateCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
  trajectory_msg.header.stamp = joint_state_msg_->header.stamp;
  trajectory_msg.joint_names = msg->name;

  auto point = trajectory_msgs::msg::JointTrajectoryPoint();
  point.positions = msg->position;
  int32_t dt_sec = 1; // 1 second
  int32_t dt_nsec = 0;  
  point.time_from_start = rclcpp::Duration(dt_sec, dt_nsec);

  trajectory_msg.points.push_back(point);

  joint_trajectory_pub_->publish(trajectory_msg);
}
} // namespace orbital_robot_simulation

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<orbital_robot_simulation::JointStateToTrajectoryNode>());
  rclcpp::shutdown();
  return 0;
}
