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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

#include <iostream>

#include "waitset_examples/visibility_control.h"

namespace waitset_examples
{
// Create a TimerListenerSingleUsingCallback class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class TimerListenerSingleUsingCallback : public rclcpp::Node
{
public:
  WAITSET_EXAMPLES_PUBLIC
  explicit TimerListenerSingleUsingCallback(const rclcpp::NodeOptions & options)
  : Node("timer_listener_single_using_callback", options)
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    // Setting parameter for the update frequency.
    double update_frequency = this->declare_parameter("update_frequency", 1.0);
    bool use_transient_local = this->declare_parameter("use_transient_local", false);

    auto executed_callback =
      [this]([[maybe_unused]] std_msgs::msg::String::ConstSharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "Catch message");

        std::ostringstream oss;
        oss << std::this_thread::get_id();
        RCLCPP_INFO(this->get_logger(), "I heard: [%s] on thread id [%s].", msg->data.c_str(), oss.str().c_str());
      };
    auto timer_callback =
        [this]() -> void
    {
      std::ostringstream oss;
      oss << std::this_thread::get_id();
      RCLCPP_INFO(this->get_logger(), "Timer triggered on thread id [%s].", oss.str().c_str());

      auto wait_result = wait_set_.wait(std::chrono::milliseconds(0));
      if (wait_result.kind() == rclcpp::WaitResultKind::Ready && 
          wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0]) {
        RCLCPP_INFO(this->get_logger(), "wait_set tells that subscription is ready");
      } else {
        RCLCPP_INFO(this->get_logger(), "wait_set tells that subscription is not ready and return");
        return;
      }

      auto msg = sub_->create_message();
      rclcpp::MessageInfo msg_info;
      if (sub_->take_type_erased(msg.get(), msg_info)) {
        sub_->handle_message(msg, msg_info);

        rmw_time_point_value_t source_timestamp =  msg_info.get_rmw_message_info().source_timestamp;

        int32_t sec = source_timestamp / 1000000000;
        uint32_t nsec = source_timestamp % 1000000000;

        RCLCPP_INFO(this->get_logger(), "Message is sent at : %d.%d", sec, nsec);
      } else {
        RCLCPP_INFO(this->get_logger(), "Not catch message");
      }
    };
    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.

    rclcpp::CallbackGroup::SharedPtr cb_group_noexec = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = cb_group_noexec;

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    if (use_transient_local) { 
      qos = qos.transient_local();
    }

    sub_ = create_subscription<std_msgs::msg::String>("chatter", qos, executed_callback, subscription_options);

    wait_set_.add_subscription(sub_);

    auto update_period = std::chrono::duration<double>(1.0 / update_frequency);
    timer_ = this->create_wall_timer(update_period, timer_callback);

    RCLCPP_INFO(this->get_logger(), "/chatter message is updated at %f Hz", update_frequency);
    if (qos.durability() == rclcpp::DurabilityPolicy::TransientLocal) {
      RCLCPP_INFO(this->get_logger(), "transient_local is enabled");
    }
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::WaitSet wait_set_;
};

}  // namespace waitset_examples

RCLCPP_COMPONENTS_REGISTER_NODE(waitset_examples::TimerListenerSingleUsingCallback)