# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        # String型のchatterトピックを送信するpublisherの定義
        self.publisher = self.create_publisher(String, 'chatter')
        # 送信周期毎にtimer_callbackを呼び出し（送信周期は0.5秒）
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World!'
        # chatterトピックにmsgを送信
        self.publisher.publish(msg)
        # msgの中身を標準出力にログ
        self.get_logger().info(msg.data)


if __name__ == '__main__':
    # Pythonクライアントライブラリの初期化
    rclpy.init(args=args)
    # minimal_publisherノードの作成
    minimal_publisher = MinimalPublisher()
    # minimal_publisherノードの実行開始
    rclpy.spin(minimal_publisher)
    # Pythonクライアントライブラリの終了
    rclpy.shutdown()
