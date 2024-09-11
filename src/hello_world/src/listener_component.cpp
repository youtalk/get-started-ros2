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

#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "hello_world/visibility_control.h"

using namespace std::chrono_literals;

// ネームスペースの設定
namespace hello_world
{

class ListenerComponent : public rclcpp::Node
{
public:
  // マルチOSに対応した共有ライブラリの最適化
  HELLO_WORLD_PUBLIC
  // コンストラクター引数をNodeOptionsに変更
  explicit ListenerComponent(const rclcpp::NodeOptions & options)
  : Node("listener_component", options)
  {
    // chatterトピックのコールバック関数
    auto callback =
      [this](const std_msgs::msg::String::UniquePtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
      };

    // chatterトピックの受信設定
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    sub_ = create_subscription<std_msgs::msg::String>(
      "chatter", qos, callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};  // class ListenerComponent

}  // namespace hello_world

#include "rclcpp_components/register_node_macro.hpp"

// クラスローダーにコンポーネントを登録
RCLCPP_COMPONENTS_REGISTER_NODE(hello_world::ListenerComponent)
