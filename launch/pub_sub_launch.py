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

"""Launch a talker and a listener."""

from launch import LaunchContext, LaunchDescription
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='ros2_cpp_py', node_executable='pub_test', output='screen',
            node_name='pub_test',
            parameters=[{"width": 1024, "height": 1024, "frame_rate": 30.0}],
            ),
        launch_ros.actions.Node(
            package='ros2_cpp_py', node_executable='sub_test', output='screen',
            node_name='sub_test',
            ),
    ])
