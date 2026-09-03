// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace ffw_follower_joint_state_relay
{

class FollowerJointStateRelay : public rclcpp::Node
{
public:
  FollowerJointStateRelay()
  : rclcpp::Node("follower_joint_state_relay")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/joint_states");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/leader/follower_joint_states");

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    publisher_ = create_publisher<sensor_msgs::msg::JointState>(output_topic_, qos);
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      input_topic_, qos,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        publisher_->publish(*msg);
        if (!first_message_forwarded_) {
          first_message_forwarded_ = true;
          RCLCPP_INFO(
            get_logger(), "Follower joint state connected: %s -> %s (%zu joints)",
            input_topic_.c_str(), output_topic_.c_str(), msg->name.size());
        }
      });

    RCLCPP_INFO(
      get_logger(), "Waiting for follower joint state: %s -> %s",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  std::string input_topic_;
  std::string output_topic_;
  bool first_message_forwarded_ = false;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

}  // namespace ffw_follower_joint_state_relay

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ffw_follower_joint_state_relay::FollowerJointStateRelay>());
  rclcpp::shutdown();
  return 0;
}
