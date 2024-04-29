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
// Create a TimerListenerSingleWaitSomePeriod class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class TimerListenerSingleWaitSomePeriod : public rclcpp::Node
{
public:
  WAITSET_EXAMPLES_PUBLIC
  explicit TimerListenerSingleWaitSomePeriod(const rclcpp::NodeOptions & options)
  : Node("timer_listener_single_wait_some_period", options)
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    // Setting parameter for the update frequency.
    double update_frequency = this->declare_parameter("update_frequency", 1.0);
    bool use_transient_local = this->declare_parameter("use_transient_local", false);
    waiting_time_ = this->declare_parameter("waiting_time", 4.0);

    auto not_executed_callback =
      [this]([[maybe_unused]] std_msgs::msg::String::ConstSharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "I never heard message");
      };
    auto timer_callback =
        [this]() -> void
    {
      RCLCPP_INFO(this->get_logger(), "Timer triggered.");

      rclcpp::Time beginning_of_waiting = this->now();
      rclcpp::Time end_of_waiting;

      auto waiting_time = std::chrono::duration<double>(waiting_time_);
      auto wait_result = wait_set_.wait(waiting_time);
      if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
        RCLCPP_INFO(this->get_logger(), "[Received] wait_set tells that subscription is ready");
        end_of_waiting = this->now();
        RCLCPP_INFO(this->get_logger(), "waiting time is %f", (end_of_waiting - beginning_of_waiting).seconds());
      } else if (wait_result.kind() == rclcpp::WaitResultKind::Timeout) {
        RCLCPP_INFO(this->get_logger(), "[Timeout] timeout is reached");
        end_of_waiting = this->now();
        RCLCPP_INFO(this->get_logger(), "waiting time is %f", (end_of_waiting - beginning_of_waiting).seconds());
        return;
      } else {
        RCLCPP_INFO(this->get_logger(), "[Failed] wait_set tells that subscription is not ready and return");
        return;
      }

      std_msgs::msg::String msg;
      rclcpp::MessageInfo msg_info;
      if (sub_->take(msg, msg_info)) {
        RCLCPP_INFO(this->get_logger(), "Catch message");
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg.data.c_str());

        rmw_time_point_value_t source_timestamp =  msg_info.get_rmw_message_info().source_timestamp;

        int32_t sec = source_timestamp / 1000000000;
        uint32_t nsec = source_timestamp % 1000000000;

        RCLCPP_INFO(this->get_logger(), "Message is sent at : %d.%d", sec, nsec);

      } else {
        assert(false);
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

    sub_ = create_subscription<std_msgs::msg::String>("slowest_chatter", qos, not_executed_callback, subscription_options);

    // Add subscription to waitset
    wait_set_.add_subscription(sub_);

    auto update_period = std::chrono::duration<double>(1.0 / update_frequency);
    timer_ = this->create_wall_timer(update_period, timer_callback);

    std::chrono::duration<double> waiting_time(waiting_time);

    // printout the settings.
    RCLCPP_INFO(this->get_logger(), "/chatter message is updated at %f Hz", update_frequency);
    if (qos.durability() == rclcpp::DurabilityPolicy::TransientLocal) {
      RCLCPP_INFO(this->get_logger(), "transient_local is enabled");
    }
    RCLCPP_INFO(this->get_logger(), "waiting time is %f", waiting_time_);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::WaitSet wait_set_;
  double waiting_time_;
  
};

}  // namespace waitset_examples

RCLCPP_COMPONENTS_REGISTER_NODE(waitset_examples::TimerListenerSingleWaitSomePeriod)
