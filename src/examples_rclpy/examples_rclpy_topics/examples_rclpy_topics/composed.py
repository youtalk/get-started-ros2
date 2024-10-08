# Copyright 2017 Open Source Robotics Foundation, Inc.
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

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor

from examples_rclpy_topics.publisher import MinimalPublisher
from examples_rclpy_topics.subscriber import MinimalSubscriber


def main(args=None):
    rclpy.init(args=args)
    try:
        publisher = MinimalPublisher()
        subscriber = MinimalSubscriber()
        # すべてのコールバックをメインスレッドで処理
        executor = SingleThreadedExecutor()
        # executorにすべてのノードを登録
        executor.add_node(publisher)
        executor.add_node(subscriber)

        try:
            # 登録されたすべてのノードを実行
            executor.spin()
        finally:
            executor.shutdown()
            subscriber.destroy_node()
            publisher.destroy_node()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
