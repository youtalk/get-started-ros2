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

from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        # add_two_intsサービスのクライアント作成
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        # add_two_intsサービスの起動待機
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting...')
        # add_two_intsサービスの引数
        self.request = AddTwoInts.request()

    def call_async(self):
        # add_two_intsサービスの引数にa = 1, b = 2を設定
        self.request.a = 1
        self.request.b = 2
        # add_two_intsサービスの非同期実行
        return self.cli.call_async(self.request)


if __name__ == '__main__':
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    future = minimal_client.call_async()
    # add_two_intsサービスの非同期実行が完了するまで待機
    rclpy.spin_until_future_complete(minimal_client, future)

    # 返り値を正常に得られたか判別
    if future.done() and future.result() is not None:
        # 返り値の取得
        response = future.result()
        # 計算結果を標準出力にログ
        minimal_client.get_logger().info(
            '%d + %d = %d' %
            (minimal_client.request.a, minimal_client.request.b,
             response.sum))

    rclpy.shutdown()
