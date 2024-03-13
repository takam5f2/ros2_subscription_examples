// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"

#include "waitset_examples/visibility_control.h"

using namespace std::chrono_literals;

namespace waitset_examples
{
// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class TalkerTriple : public rclcpp::Node
{
public:
  WAITSET_EXAMPLES_PUBLIC
  explicit TalkerTriple(const rclcpp::NodeOptions & options)
  : Node("talker_triple", options)
  {
    // Create a function for when messages are to be sent.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Parameter settings for the update frequency.
    double update_frequency = this->declare_parameter("update_frequency", 1.0);
    bool use_transient_local = this->declare_parameter("use_transient_local", false);

    auto publish_message =
      [this]() -> void
      {
        msg_ = std::make_unique<std_msgs::msg::String>();
        msg_->data = "Good Morning: " + std::to_string(count_++);
        if (this->get_node_options().use_intra_process_comms()) {
          RCLCPP_INFO(this->get_logger(), "Publishing: '%s' (via intra)", msg_->data.c_str());
        } else {
          RCLCPP_INFO(this->get_logger(), "Publishing: '%s' (via inter)", msg_->data.c_str());
        }
        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        pub_->publish(std::move(msg_));
      };
    auto slower_publish_message = [this]() -> void
      {
        if (slower_pub_->get_subscription_count() == 0) {
          return;
        }
        slower_msg_ = std::make_unique<std_msgs::msg::String>();
        slower_msg_->data = "Good Afternoon: " + std::to_string(slower_count_++);
        if (this->get_node_options().use_intra_process_comms()) {
          RCLCPP_INFO(this->get_logger(), "Publishing: '%s' (via intra)", slower_msg_->data.c_str());
        } else {
          RCLCPP_INFO(this->get_logger(), "Publishing: '%s' (via inter)", slower_msg_->data.c_str());
        }
        slower_pub_->publish(std::move(slower_msg_));
      };
    auto slowest_publish_message = [this]() -> void
      {
        if (slowest_pub_->get_subscription_count() == 0) {
          return;
        }
        slowest_msg_ = std::make_unique<std_msgs::msg::String>();
        slowest_msg_->data = "Good Night: " + std::to_string(slowest_count_++);
        if (this->get_node_options().use_intra_process_comms()) {
          RCLCPP_INFO(this->get_logger(), "Publishing: '%s' (via intra)", slowest_msg_->data.c_str());
        } else {
          RCLCPP_INFO(this->get_logger(), "Publishing: '%s' (via inter)", slowest_msg_->data.c_str());
        }
        slowest_pub_->publish(std::move(slowest_msg_));
      };

    // Create a publisher with a custom Quality of Service profile.
    // Uniform initialization is suggested so it can be trivially changed to
    // rclcpp::KeepAll{} if the user wishes.
    // (rclcpp::KeepLast(7) -> rclcpp::KeepAll() fails to compile)
    rclcpp::QoS qos(rclcpp::KeepLast{10});
    if (use_transient_local) {
      qos = qos.transient_local();
    }

    pub_ = this->create_publisher<std_msgs::msg::String>("chatter", qos);
    slower_pub_ = this->create_publisher<std_msgs::msg::String>("slower_chatter", qos);
    slowest_pub_ = this->create_publisher<std_msgs::msg::String>("slowest_chatter", qos);

    // Use a timer to schedule periodic message publishing.
    // The callback function is called each time the timer expires.
    // The call is non-blocking.
    auto period = std::chrono::duration<double>(1.0 / update_frequency);
    timer_ = this->create_wall_timer(period, publish_message);
    slower_timer_ = this->create_wall_timer(period * 2, slower_publish_message);
    slowest_timer_ = this->create_wall_timer(period * 3, slowest_publish_message);
  }

private:
  size_t count_ = 1;
  size_t slower_count_ = 1;
  size_t slowest_count_ = 1;
  std::unique_ptr<std_msgs::msg::String> msg_;
  std::unique_ptr<std_msgs::msg::String> slower_msg_;
  std::unique_ptr<std_msgs::msg::String> slowest_msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr slower_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr slowest_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr slower_timer_;
  rclcpp::TimerBase::SharedPtr slowest_timer_;
};

}  // namespace waitset_examples

RCLCPP_COMPONENTS_REGISTER_NODE(waitset_examples::TalkerTriple)
