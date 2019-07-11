# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import time

from example_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class MinimalActionServer(Node):
    def __init__(self):
        super().__init__('minimal_action_server')
        # fibonacciアクションサーバーの作成
        # （execute_callback実行は複数同時処理を許可）
        self._action_server = ActionServer(
            self, Fibonacci, 'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup())

    def destroy(self):
        # アクションサーバーの終了
        self._action_server.destroy()
        super().destroy_node()

    async def execute_callback(self, goal_handle):
        # アクションの実行
        self.get_logger().info('executing...')
        # フィボナッチ数列の初期値0, 1を設定
        msg = Fibonacci.Feedback()
        msg.sequence = [0, 1]

        # フィボナッチ数列を一つずつ追加
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                # アクションのキャンセル
                goal_handle.set_canceled()
                self.get_logger().info('goal canceled')
                return Fibonacci.Result()

            # フィボナッチ数列の更新
            msg.sequence.append(msg.sequence[i] + msg.sequence[i-1])
            self.get_logger().info('feedback: {0}'.format(msg.sequence))
            # アクションのフィードバックの送信
            goal_handle.publish_feedback(msg)
            # 1秒待機（重たい処理の代わり）
            time.sleep(1)

        # アクションの実行結果の送信
        goal_handle.set_succeeded()
        result = Fibonacci.Result()
        result.sequence = msg.sequence
        self.get_logger().info('result: {0}'.format(result.sequence))
        return result


if __name__ == '__main__':
    rclpy.init(args=args)
    minimal_action_server = MinimalActionServer()
    # マルチスレッドでminimal_action_serverノードを実行し、
    # 複数のアクションクライアントを同時処理
    executor = MultiThreadedExecutor()
    rclpy.spin(minimal_action_server, executor=executor)
    minimal_action_server.destroy()
    rclpy.shutdown()
