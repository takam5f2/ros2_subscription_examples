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


#include "waitset_examples/visibility_control.h"

namespace waitset_examples
{
// Create a TimerListenerTripleSeparatedWaitSet class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class TimerListenerTripleSeparatedWaitSet : public rclcpp::Node
{
public:
  WAITSET_EXAMPLES_PUBLIC
  explicit TimerListenerTripleSeparatedWaitSet(const rclcpp::NodeOptions & options)
  : Node("timer_listener_triple_separated_waitset", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    // Setting parameter for the update frequency.
    double update_frequency = this->declare_parameter("update_frequency", 1.0);
    bool use_transient_local = this->declare_parameter("use_transient_local", false);

    auto not_executed_callback =
      [this]([[maybe_unused]] std_msgs::msg::String::ConstSharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "I never heard message");
      };
    auto timer_callback =
        [this]() -> void
    {
      RCLCPP_INFO(this->get_logger(), "Normal Timer triggered.");

      auto wait_result = wait_set_.wait(std::chrono::milliseconds(0));

      if (wait_result.kind() == rclcpp::WaitResultKind::Ready && 
          wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0]) {
        std_msgs::msg::String msg;
        rclcpp::MessageInfo msg_info;
        while (sub_->take(msg, msg_info)) {
          printoutMessage("normal", msg, msg_info);
        }
      } else {
        RCLCPP_INFO(this->get_logger(), "wait_set tells that subscription is not ready and return");
        return;
      }
    };

    auto slower_timer_callback =
        [this]() -> void
    {
      RCLCPP_INFO(this->get_logger(), "Slower Timer triggered.");

      auto wait_result = slower_wait_set_.wait(std::chrono::milliseconds(0));

      if (wait_result.kind() == rclcpp::WaitResultKind::Ready && 
          wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0]) {
        std_msgs::msg::String msg;
        rclcpp::MessageInfo msg_info;
        while (slower_sub_->take(msg, msg_info)) {
          printoutMessage("slower", msg, msg_info);
        }
      } else {
        RCLCPP_INFO(this->get_logger(), "wait_set tells that subscription is not ready and return");
        return;
      }
    };

    auto slowest_timer_callback =
        [this]() -> void
    {
      RCLCPP_INFO(this->get_logger(), "Slowest Timer triggered.");

      auto wait_result = slowest_wait_set_.wait(std::chrono::milliseconds(0));

      if (wait_result.kind() == rclcpp::WaitResultKind::Ready && 
          wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0]) {
        std_msgs::msg::String msg;
        rclcpp::MessageInfo msg_info;
        while (slowest_sub_->take(msg, msg_info)) {
          printoutMessage("slowest", msg, msg_info);
        }
      } else {
        RCLCPP_INFO(this->get_logger(), "wait_set tells that subscription is not ready and return");
        return;
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

    // Create subscriptions
    sub_ = create_subscription<std_msgs::msg::String>("chatter", qos, not_executed_callback, subscription_options);
    wait_set_.add_subscription(sub_);

    slower_sub_ = create_subscription<std_msgs::msg::String>("slower_chatter", qos, not_executed_callback, subscription_options);
    slower_wait_set_.add_subscription(slower_sub_);

    slowest_sub_ = create_subscription<std_msgs::msg::String>("slowest_chatter", qos, not_executed_callback, subscription_options);
    slowest_wait_set_.add_subscription(slowest_sub_);

    // Create timers that calls the callback at the specified rate.
    auto update_period = std::chrono::duration<double>(1.0 / update_frequency);
    timer_ = this->create_wall_timer(update_period, timer_callback);
    slower_timer_ = this->create_wall_timer(update_period*3, slower_timer_callback);
    slowest_timer_ = this->create_wall_timer(update_period*4, slowest_timer_callback);

    // printout the settings.
    RCLCPP_INFO(this->get_logger(), "/chatter message is updated at %f Hz", update_frequency);
    if (qos.durability() == rclcpp::DurabilityPolicy::TransientLocal) {
      RCLCPP_INFO(this->get_logger(), "transient_local is enabled");
    }
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr slower_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr slowest_sub_;

  rclcpp::WaitSet wait_set_;
  rclcpp::WaitSet slower_wait_set_;
  rclcpp::WaitSet slowest_wait_set_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr slower_timer_;
  rclcpp::TimerBase::SharedPtr slowest_timer_;

  void printoutMessage(const std::string & prefix, const std_msgs::msg::String & msg, const rclcpp::MessageInfo & msg_info) {
    RCLCPP_INFO(this->get_logger(), "Message is received from %s timer", prefix.c_str());
    RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg.data.c_str());

    rmw_time_point_value_t source_timestamp =  msg_info.get_rmw_message_info().source_timestamp;

    int32_t sec = source_timestamp / 1000000000;
    uint32_t nsec = source_timestamp % 1000000000;

    RCLCPP_INFO(this->get_logger(), "Message is sent at : %d.%d", sec, nsec);

  }
};

}  // namespace waitset_examples

RCLCPP_COMPONENTS_REGISTER_NODE(waitset_examples::TimerListenerTripleSeparatedWaitSet)
