// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <iostream>
#include <memory>

#include "rcl/types.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"

#include "rosidl_typesupport_cpp/message_type_support.hpp"

#include "simple_examples/visibility_control.h"

namespace simple_examples
{

class TimerSerializedMessageListener : public rclcpp::Node
{
public:
  SIMPLE_EXAMPLES_PUBLIC
  explicit TimerSerializedMessageListener(const rclcpp::NodeOptions & options)
  : Node("timer_serialized_message_listener", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    double update_frequency = this->declare_parameter("update_frequency", 1.0);
    int queue_size = this->declare_parameter("queue_size", 10);
    bool use_transient_local = this->declare_parameter("use_transient_local", false);

    auto callback =
        []([[maybe_unused]] const std::shared_ptr<rclcpp::SerializedMessage> msg) -> void
    {
      assert(false);
    };
    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.
    rclcpp::QoS qos(queue_size);
    if (use_transient_local) {
      qos = qos.transient_local();
    }

    rclcpp::CallbackGroup::SharedPtr cb_group_noexec = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = cb_group_noexec;

    sub_ = create_subscription<std_msgs::msg::String>("chatter", qos , callback, subscription_options);

    auto update_period = std::chrono::milliseconds(static_cast<int>(1000 / update_frequency));
    timer_ = create_wall_timer(update_period, [this]() -> void {
      RCLCPP_INFO(this->get_logger(), "Timer triggered.");

      // receive the serialized message.
      rclcpp::MessageInfo msg_info;
      auto msg = sub_->create_serialized_message();

      if (sub_->take_serialized(*msg, msg_info) == false) {
        return;
      }

      // Print the serialized data message in HEX representation
      // This output corresponds to what you would see in e.g. Wireshark
      // when tracing the RTPS packets.
      std::cout << "I heard data of length: " << msg->size() << std::endl;
      for (size_t i = 0; i < msg->size(); ++i) {
        printf("%02x ", msg->get_rcl_serialized_message().buffer[i]);
      }
      printf("\n");

      // In order to deserialize the message we have to manually create a ROS2
      // message in which we want to convert the serialized data.
      using MessageT = std_msgs::msg::String;
      MessageT string_msg;
      auto serializer = rclcpp::Serialization<MessageT>();
      serializer.deserialize_message(msg.get(), &string_msg);
      // Finally print the ROS2 message data
      std::cout << "serialized data after deserialization: " << string_msg.data << std::endl;

    });
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace simple_examples

RCLCPP_COMPONENTS_REGISTER_NODE(simple_examples::TimerSerializedMessageListener)
