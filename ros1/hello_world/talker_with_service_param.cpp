/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <hello_world/SetMessage.h>

// 文字列の送信データ
std_msgs::String msg;

// set_messageサービスのコールバック関数
bool SetMessage(hello_world::SetMessage::Request &req,
                hello_world::SetMessage::Response &res)
{
  ROS_INFO("message %s -> %s", msg.data.c_str(), req.message.c_str());
  // 文字列の変更
  msg.data = req.message;
  // 返り値をtrueに設定
  res.result = true;
  return true;
}

int main(int argc, char **argv)
{
  // talkerノードの初期化
  ros::init(argc, argv, "talker");
  // ノードの本体
  ros::NodeHandle n;
  // set_messageサービスの登録とコールバック関数の登録
  ros::ServiceServer service = n.advertiseService(
      "set_message", SetMessage);
  // chatterトピックを送信する送信器
  ros::Publisher chatter = n.advertise<std_msgs::String>(
      "chatter", 1000);
  // 10Hzの送信周期
  ros::Rate loop_rate(10);

  msg.data = "Hello world!";

  // 強制終了していないか確認
  while (ros::ok())
  {
    // decorationパラメーターの取得
    std::string decoration = "";
    n.param<std::string>("decoration", decoration, "");
    std::string decorated_data = decoration + msg.data + decoration;
    ROS_INFO("%s", decorated_data.c_str());
    // 文字列の送信
    chatter.publish(msg);
    // ノードの処理サイクルを1回分進行
    ros::spinOnce();
    // 10Hzの送信周期になるように待機
    loop_rate.sleep();
  }

  return 0;
}
