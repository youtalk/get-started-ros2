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
#include <rclcpp/parameter_value.hpp>
#include <string>
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

        // decorationによる文字列の装飾
        std::string decorated_data =
          decoration_ + msg->data + decoration_;
        RCLCPP_INFO(this->get_logger(), "%s", decorated_data.c_str());
        pub_->publish(std::move(msg));
      };

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    pub_ = create_publisher<std_msgs::msg::String>(topic_name, qos);
    timer_ = create_wall_timer(100ms, publish_message);

    auto handle_set_message =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<SetMessage::Request> request,
      std::shared_ptr<SetMessage::Response> response) -> void
      {
        (void)request_header;
        RCLCPP_INFO(this->get_logger(), "message %s -> %s",
                    this->data_.c_str(), request->message.c_str());
        this->data_ = request->message;
        response->result = true;
      };

    srv_ = create_service<SetMessage>(
      "set_message", handle_set_message);

    // decorationパラメータの宣言
    decoration_ = declare_parameter("decoration", "");
    // decorationパラメータの監視
    param_subscriber_ = std::make_shared<
      rclcpp::ParameterEventHandler>(this);
    cb_handle_ = param_subscriber_->add_parameter_callback(
        "decoration", [this](const rclcpp::Parameter & p) {
        decoration_ = p.as_string();
        });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<SetMessage>::SharedPtr srv_;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
  std::string data_;
  std::string decoration_;
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
