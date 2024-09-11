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
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "hello_world_msgs/srv/set_message.hpp"

using namespace std::chrono_literals;
using hello_world_msgs::srv::SetMessage;

class Talker : public rclcpp::Node
{
public:
  explicit Talker(const std::string & topic_name)
  : Node("talker"),
    data_("Hello world!")
  {
    auto publish_message =
      [this]() -> void
      {
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = data_;

        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
        pub_->publish(std::move(msg));
      };

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    pub_ = create_publisher<std_msgs::msg::String>(topic_name, qos);
    timer_ = create_wall_timer(100ms, publish_message);

    // set_messageサービスのコールバック関数
    auto handle_set_message =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<SetMessage::Request> request,
      std::shared_ptr<SetMessage::Response> response) -> void
      {
        (void)request_header;  // Lintツール対策
        RCLCPP_INFO(this->get_logger(), "message %s -> %s",
                    this->data_.c_str(), request->message.c_str());
        // 1秒スリープ（重い処理の代わり）
        std::this_thread::sleep_for(1s);
        this->data_ = request->message;
        response->result = true;
      };

    // set_messageサービスのサーバー設定
    srv_ = create_service<SetMessage>(
      "set_message", handle_set_message);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<SetMessage>::SharedPtr srv_;
  std::string data_;
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Talker>("chatter");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
