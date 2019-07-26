# Copyright 2019 Yutaka Kondo
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

import unittest

from launch import LaunchDescription
import launch_ros.actions


class TestTalkerListener(unittest.TestCase):
    def _assert_launch_errors(self, actions):
        ld = LaunchDescription(actions)
        ls = LaunchService()
        ls.include_launch_description(ld)
        assert 0 != ls.run()

    def _assert_launch_no_errors(self, actions):
        ld = LaunchDescription(actions)
        ls = LaunchService()
        ls.include_launch_description(ld)
        assert 0 == ls.run()

    def test_launch(self):
        self._assert_launch_no_errors([
            launch_ros.actions.Node(
                package='hello_world', node_executable='talker',
                output='screen'),
            launch_ros.actions.Node(
                package='hello_world', node_executable='listener',
                output='screen'),
        ])
