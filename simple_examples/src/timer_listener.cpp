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


#include "simple_examples/visibility_control.h"

namespace simple_examples
{
// Create a TimerListener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class TimerListener : public rclcpp::Node
{
public:
  SIMPLE_EXAMPLES_PUBLIC
  explicit TimerListener(const rclcpp::NodeOptions & options)
  : Node("timer_listener", options)
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    // Setting parameter for the update frequency.
    double update_frequency = this->declare_parameter("update_frequency", 1.0);
    bool use_transient_local = this->declare_parameter("use_transient_local", false);
    printout_message_info_ = this->declare_parameter("printout_message_info", false);

    auto not_executed_callback =
      [this]([[maybe_unused]] std_msgs::msg::String::ConstSharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "I never heard message");
      };
    auto timer_callback =
        [this]() -> void
    {
      RCLCPP_INFO(this->get_logger(), "Timer triggered.");
      std_msgs::msg::String msg;
      rclcpp::MessageInfo msg_info;
      if (sub_->take(msg, msg_info)) {
        RCLCPP_INFO(this->get_logger(), "Catch message");
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg.data.c_str());

        // printout the message info
        if (printout_message_info_) {
          printout_message_info(msg_info);
        }

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

    sub_ = create_subscription<std_msgs::msg::String>("chatter", qos, not_executed_callback, subscription_options);

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
  bool printout_message_info_ = false;

  void printout_message_info (const rclcpp::MessageInfo & msg_info) {
    // print out the source timestamp
    rmw_time_point_value_t source_timestamp =  msg_info.get_rmw_message_info().source_timestamp;
    const int32_t source_timestamp_sec = source_timestamp / 1000000000;
    const uint32_t source_timestamp_nsec = source_timestamp % 1000000000;
    RCLCPP_INFO(this->get_logger(), "Message is sent at : %d.%d", source_timestamp_sec, source_timestamp_nsec);

    rmw_time_point_value_t received_timestamp =  msg_info.get_rmw_message_info().received_timestamp;
    const int32_t received_timestamp_sec = received_timestamp / 1000000000;
    const uint32_t received_timestamp_nsec = received_timestamp % 1000000000;
    RCLCPP_INFO(this->get_logger(), "Message is received at : %d.%d", received_timestamp_sec, received_timestamp_nsec);

    RCLCPP_INFO(this->get_logger(), "publication sequence number: %ld", msg_info.get_rmw_message_info().publication_sequence_number);
    RCLCPP_INFO(this->get_logger(), "reception sequence number: %ld", msg_info.get_rmw_message_info().reception_sequence_number);

    std::ostringstream oss;
    for (size_t i = sizeof(msg_info.get_rmw_message_info().publisher_gid.data); i > 0 ; i--)
    {
      oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(msg_info.get_rmw_message_info().publisher_gid.data[i-1]); // This is not good code, but it is a simple example to understand the publisher_gid.
    }

      RCLCPP_INFO(this->get_logger(), "Message is sent from node: %s", oss.str().c_str());

    if (msg_info.get_rmw_message_info().from_intra_process) {
      RCLCPP_INFO(this->get_logger(), "Message is sent from intra process");
    } else {
      RCLCPP_INFO(this->get_logger(), "Message is sent from inter process");
    }
    if (printout_message_info_) {
      RCLCPP_INFO(this->get_logger(), "Message info is printed out");
    }
  }

};

}  // namespace simple_examples

RCLCPP_COMPONENTS_REGISTER_NODE(simple_examples::TimerListener)
