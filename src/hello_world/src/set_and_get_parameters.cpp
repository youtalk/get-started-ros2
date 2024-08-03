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

#include <memory>
#include <rclcpp/parameter_value.hpp>
#include <sstream>
#include <vector>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("set_and_get_parameters");

  // パラメータの宣言
  node->declare_parameter("foo", rclcpp::PARAMETER_INTEGER);
  node->declare_parameter("bar", rclcpp::PARAMETER_STRING);
  node->declare_parameter("baz", rclcpp::PARAMETER_DOUBLE);

  // パラメータ設定・取得サービスのクライアント
  auto parameters_client = std::make_shared<
    rclcpp::SyncParametersClient>(node);
  // パラメータ設定・取得サービスの起動待ち
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Waiting");
  }

  // パラメータの設定
  auto set_parameters_results = parameters_client->set_parameters({
    rclcpp::Parameter("foo", 2),
    rclcpp::Parameter("bar", "hello"),
    rclcpp::Parameter("baz", 1.45),
  });
  // パラメータの設定成功の確認
  for (auto & result : set_parameters_results) {
    if (!result.successful) {
      RCLCPP_ERROR(node->get_logger(), "Failed: %s",
                   result.reason.c_str());
    }
  }

  std::stringstream ss;
  // パラメータの取得
  for (auto & parameter : parameters_client->get_parameters(
      {"foo", "bar", "baz"}))
  {
    // パラメータ名とパラメータの型名のロギング
    ss << "\nParameter name: " << parameter.get_name();
    ss << "\nParameter value (" << parameter.get_type_name()
       << "): " << parameter.value_to_string();
  }
  RCLCPP_INFO(node->get_logger(), ss.str().c_str());

  rclcpp::shutdown();
  return 0;
}
