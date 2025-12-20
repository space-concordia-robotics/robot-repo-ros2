#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>


void goTarget(double x, double y, double z, 
              moveit::planning_interface::MoveGroupInterface& move_group,
              rclcpp::Node::SharedPtr move_group_node){
  //define pose target
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;

  move_group.setPoseTarget(target_pose);

  // Plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    auto error = move_group.plan(my_plan);

    if (error.val == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(move_group_node->get_logger(), "Plan successful!");
        move_group.move();
    } else {
        RCLCPP_WARN(move_group_node->get_logger(), "Planning failed!");
    }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  static const std::string PLANNING_GROUP = "ceres_rover"; //name of the arm in SRDF file
  
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_node");
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  
  std::string letter = "A";

  move_group_node->declare_parameter<double>(letter + ".x", 0.0);
  move_group_node->declare_parameter<double>(letter + ".y", 0.0);
  move_group_node->declare_parameter<double>(letter + ".z", 0.0);
  double x = move_group_node->get_parameter(letter + ".x").as_double();
  double y = move_group_node->get_parameter(letter + ".y").as_double();
  double z = move_group_node->get_parameter(letter + ".z").as_double();
  RCLCPP_INFO(move_group_node->get_logger(), "Target coordinates: x=%f, y=%f, z=%f", x, y, z);

  goTarget(x, y, z, move_group, move_group_node);

  rclcpp::shutdown();
  return 0;
}