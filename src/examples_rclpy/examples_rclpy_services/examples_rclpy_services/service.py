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


class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        # add_two_intsサービスの作成
        # （サービスの中身はadd_two_ints_callbackで実装）
        self.srv = self.create_service(AddTwoInts, 'add_two_ints',
                                       self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        # a + bを答えを返り値responseに設定
        response.sum = request.a + request.b
        # 計算結果を標準出力にログ
        self.get_logger().info('%d + %d = %d' %
                               (request.a, request.b, response.sum))
        return response


def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
