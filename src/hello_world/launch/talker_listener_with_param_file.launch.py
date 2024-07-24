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

from launch import LaunchDescription
import launch.substitutions
import launch_ros.actions
import os


def generate_launch_description():
    params = os.path.join(os.path.dirname(__file__), 'params.yaml')

    return LaunchDescription([
        launch_ros.actions.Node(
            package='hello_world', namespace='hello_world',
            executable='talker_with_service_param',
            name='talker', output='screen',
            parameters=[params]),
        launch_ros.actions.Node(
            package='hello_world', namespace='hello_world',
            executable='listener', name='listener',
            output='screen'),
    ])
