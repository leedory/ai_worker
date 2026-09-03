// Copyright 2025 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace ffw_gripper_trigger
{

class GripperTrigger : public rclcpp::Node
{
public:
  GripperTrigger()
  : rclcpp::Node("gripper_trigger")
  {
    press_threshold_ = declare_parameter<double>("press_threshold", -0.7);
    save_pose_id_ = static_cast<uint8_t>(
      declare_parameter<int>("save_pose_id", 3));

    leader_joint_states_topic_ = declare_parameter<std::string>(
      "leader_joint_states_topic", "/leader/joint_states");

    left_enable_topic_ = declare_parameter<std::string>(
      "left_enable_topic", "/leader/left_command");
    right_enable_topic_ = declare_parameter<std::string>(
      "right_enable_topic", "/leader/right_command");

    left_gripper_joint_ = declare_parameter<std::string>(
      "left_gripper_joint", "gripper_l_joint1");
    right_gripper_joint_ = declare_parameter<std::string>(
      "right_gripper_joint", "gripper_r_joint1");

    auto qos = rclcpp::SystemDefaultsQoS();
    left_enable_pub_ = create_publisher<std_msgs::msg::UInt8>(left_enable_topic_, qos);
    right_enable_pub_ = create_publisher<std_msgs::msg::UInt8>(right_enable_topic_, qos);

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      leader_joint_states_topic_, rclcpp::SystemDefaultsQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        double left_position = std::numeric_limits<double>::quiet_NaN();
        double right_position = std::numeric_limits<double>::quiet_NaN();
        if (find_joint_position(*msg, left_gripper_joint_, left_position)) {
          check_edge(left_position, left_below_, left_enable_pub_, "left");
        }
        if (find_joint_position(*msg, right_gripper_joint_, right_position)) {
          check_edge(right_position, right_below_, right_enable_pub_, "right");
        }
      });

    RCLCPP_INFO(get_logger(),
      "gripper_trigger started (topic=%s, threshold=%.2f, home_pose_id=%u)",
      leader_joint_states_topic_.c_str(), press_threshold_, save_pose_id_);
  }

private:
  bool find_joint_position(
    const sensor_msgs::msg::JointState & msg, const std::string & joint_name,
    double & position) const
  {
    const auto it = std::find(msg.name.begin(), msg.name.end(), joint_name);
    if (it == msg.name.end()) {
      return false;
    }
    const auto index = static_cast<size_t>(std::distance(msg.name.begin(), it));
    if (index >= msg.position.size()) {
      return false;
    }
    position = msg.position[index];
    return !std::isnan(position);
  }

  // The negative (push) direction requests home once. The positive (pull)
  // direction remains part of the normal gripper trajectory.
  void check_edge(
    double gripper_pos,
    bool & below,
    const rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr & enable_pub,
    const char * tag)
  {
    if (std::isnan(gripper_pos)) {
      return;
    }

    const bool below_now = gripper_pos < press_threshold_;
    if (below_now && !below) {
      std_msgs::msg::UInt8 msg;
      msg.data = save_pose_id_;
      enable_pub->publish(msg);
      RCLCPP_WARN(get_logger(),
        "[%s] Trigger pushed (%.2f < %.2f); requested home pose %u",
        tag, gripper_pos, press_threshold_, save_pose_id_);
    }
    below = below_now;
  }

  // Params
  double press_threshold_;
  uint8_t save_pose_id_;
  std::string leader_joint_states_topic_;
  std::string left_enable_topic_;
  std::string right_enable_topic_;
  std::string left_gripper_joint_;
  std::string right_gripper_joint_;

  // Edge state per side
  bool left_below_ = false;
  bool right_below_ = false;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr left_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr right_enable_pub_;
};

}  // namespace ffw_gripper_trigger

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ffw_gripper_trigger::GripperTrigger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
