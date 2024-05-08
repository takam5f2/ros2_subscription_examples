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


#include "intra_process_talker_listener/visibility_control.h"

namespace intra_process_talker_listener
{
// Create a TimerListener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class TimerListenerIntraProcess : public rclcpp::Node
{
public:
  INTRA_PROCESS_TALKER_LISTENER_PUBLIC
  explicit TimerListenerIntraProcess(const rclcpp::NodeOptions & options)
  : Node("timer_listener_intra_process", options)
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    // Setting parameter for the update frequency.
    double update_frequency = this->declare_parameter("update_frequency", 1.0);
    bool use_transient_local = this->declare_parameter("use_transient_local", false);

    auto subscription_callback =
      [this]([[maybe_unused]] std_msgs::msg::String::ConstSharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "Catch message");
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
      };
    auto timer_callback =
        [this]() -> void
    {
      RCLCPP_INFO(this->get_logger(), "Timer triggered.");

      // check if intra-process communication is enabled.
      if (this->get_node_options().use_intra_process_comms()){

        // get the intra-process subscription's waitable.
        auto intra_process_sub = sub_->get_intra_process_waitable();

        // check if the waitable has data.
        if (intra_process_sub->is_ready(nullptr) == true) {

          // take the data and execute the callback.
          std::shared_ptr<void> data = intra_process_sub->take_data();

          RCLCPP_INFO(this->get_logger(), " Intra-process communication is performed.");

          // execute the callback.
          intra_process_sub->execute(data);
        } else {
          RCLCPP_INFO(this->get_logger(), " timer does not call sub_ callback due to no data by intra-process communication.");
        }
      } else {
        // get the message and message info.
        auto msg = sub_->create_message();
        rclcpp::MessageInfo msg_info;

        // check if data is taken from the subscription.
        if (sub_->take_type_erased(msg.get(), msg_info)) {

          RCLCPP_INFO(this->get_logger(), " Inter-process communication is performed.");

          // execute the callback.
          sub_->handle_message(msg, msg_info);

          // printout the source timestamp.
          rmw_time_point_value_t source_timestamp =  msg_info.get_rmw_message_info().source_timestamp;

          int32_t sec = source_timestamp / 1000000000;
          uint32_t nsec = source_timestamp % 1000000000;

          RCLCPP_INFO(this->get_logger(), "Message is sent at : %d.%d", sec, nsec);

        } else {
          std::cout << " timer does not call sub_ callback due to no data by inter-process communication." << std::endl;
        }
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

    // subscription.
    sub_ = create_subscription<std_msgs::msg::String>("chatter", qos, subscription_callback, subscription_options);

    // timer.
    auto update_period = std::chrono::duration<double>(1.0 / update_frequency);
    timer_ = this->create_wall_timer(update_period, timer_callback);

    // printout the node and qos settings.
    RCLCPP_INFO(this->get_logger(), "/chatter message is updated at %f Hz", update_frequency);
    if (qos.durability() == rclcpp::DurabilityPolicy::TransientLocal) {
      RCLCPP_INFO(this->get_logger(), "transient_local is enabled");
    }
    if (this->get_node_options().use_intra_process_comms()) {
      RCLCPP_INFO(this->get_logger(), "Intra-process communication is enabled.");
    }

  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

};

}  // namespace intra_process_talker_listener

RCLCPP_COMPONENTS_REGISTER_NODE(intra_process_talker_listener::TimerListenerIntraProcess)
