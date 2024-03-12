// Copyright 2015 Open Source Robotics Foundation, Inc.
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
#include <cinttypes>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

// Node that produces messages.
struct Producer : public rclcpp::Node
{
  Producer(const std::string & name, const std::string & output)
  : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    // Create a publisher on the output topic.
    pub_ = this->create_publisher<std_msgs::msg::Int32>(output, 10);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
    // Create a timer which publishes on the output topic at ~1Hz.
    auto callback = [captured_pub]() -> void {
        auto pub_ptr = captured_pub.lock();
        if (!pub_ptr) {
          return;
        }
        static int32_t count = 0;
        std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32());
        msg->data = count++;
        printf(
          "Published message with value: %d, and address: 0x%" PRIXPTR "\n", msg->data,
          reinterpret_cast<std::uintptr_t>(msg.get()));
        pub_ptr->publish(std::move(msg));
      };
    timer_ = this->create_wall_timer(1s, callback);
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#include <chrono>
#include <memory>
using namespace std::chrono_literals;


// Node that consumes messages.
struct Consumer : public rclcpp::Node
{
  Consumer(const std::string & name, const std::string & input)
  : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    // Create a subscription on the input topic which prints on receipt of new messages.
    rclcpp::CallbackGroup::SharedPtr cb_group_noexec = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = cb_group_noexec;

    sub_ = this->create_subscription<std_msgs::msg::Int32>(
        input,
        10,
        [](std_msgs::msg::Int32::UniquePtr msg) 
        {
          printf(
              " Received message with value: %d, and address: 0x%" PRIXPTR "\n", msg->data,
              reinterpret_cast<std::uintptr_t>(msg.get()));
          std::cout << " sub   callback tid = " << std::this_thread::get_id() << std::endl;
        },
        subscription_options
        );

    timer_ = this->create_wall_timer(500ms, [this]() {
      std::cout << " timer callback tid = " << std::this_thread::get_id() << std::endl;
      if (this->get_node_options().use_intra_process_comms()) {
        auto intra_process_sub = sub_->get_intra_process_waitable();
        if (intra_process_sub->is_ready(nullptr) == true) {
          std::cout << " Intra-process communication is performed." << std::endl;
          std::shared_ptr<void> data = intra_process_sub->take_data();
          intra_process_sub->execute(data);
        } else {
          std::cout << " timer does not call sub_ callback due to no data by intra-process communication." << std::endl;
        }
      } else {
        auto msg = sub_->create_message();
        rclcpp::MessageInfo msg_info;
        if (sub_->take_type_erased(msg.get(), msg_info)) {
          std::cout << " Inter-process communication is performed." << std::endl;
          sub_->handle_message(msg, msg_info);
        } else {
          std::cout << " timer does not call sub_ callback due to no data by inter-process communication." << std::endl;
        }
      }
      std::cout << " timer callback tid = " << std::this_thread::get_id() << std::endl;
      });

  }
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  std::vector<rclcpp::Parameter> params = { rclcpp::Parameter("use_intra_process_comms", true) };
  rclcpp::NodeOptions options;
  options.parameter_overrides(params);

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto producer = std::make_shared<Producer>("producer", "number");
  auto consumer = std::make_shared<Consumer>("consumer", "number");

  executor.add_node(producer);
  executor.add_node(consumer);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
