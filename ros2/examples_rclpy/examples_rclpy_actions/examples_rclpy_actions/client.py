# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from action_msgs.msg import GoalStatus
from example_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class MinimalActionClient(Node):
    def __init__(self):
        super().__init__('minimal_action_client')
        # fibonacciアクションクライアントを作成
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def goal_response_callback(self, future):
        # 目標値の設定成功の判別
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('goal rejected')
            return

        # アクションの実行結果の受信
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            self.get_result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info('feedback: {0}'.format(
            feedback.sequence))

    def get_result_callback(self, future):
        # アクションの実行状態の取得
        status = future.result().action_status
        if status == GoalStatus.STATUS_SUCCEEDED:
            # 実行成功ならフィボナッチ数列を標準出力にログ
            self.get_logger().info('result: {0}'.format(
                future.result().sequence))

        # プログラムの終了
        rclpy.shutdown()

    def send_goal(self):
        self.get_logger().info('waiting...')
        self._action_client.wait_for_server()
        # 10要素のフィボナッチ数列を標値に設定
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10
        # アクションの非同期実行
        # （フィードバックと実行結果の受信コールバック関数も設定）
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(
            self.goal_response_callback)


if __name__ == '__main__':
    rclpy.init(args=args)
    action_client = MinimalActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)
    action_client.destroy_node()
