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
        launch.actions.DeclareLaunchArgument(
            'width',
            default_value='1024',
            description='generated image width',
            ),
        launch.actions.DeclareLaunchArgument(
            'height',
            default_value='1024',
            description='generated image height',
            ),
        launch.actions.DeclareLaunchArgument(
            'frame_rate',
            default_value='30.0',
            description='publish frame rate',
            ),
        launch_ros.actions.Node(
            package='ros2_cpp_py', node_executable='ros2_pub_test', output='screen',
            node_name='ros2_pub_test',
            parameters=[{
                "width": launch.substitutions.LaunchConfiguration('width'),
                "height": launch.substitutions.LaunchConfiguration('height'),
                "frame_rate": launch.substitutions.LaunchConfiguration('frame_rate'),
                }],
            ),
        launch_ros.actions.Node(
            package='ros2_cpp_py', node_executable='ros2_sub_test', output='screen',
            node_name='ros2_sub_test',
            ),
    ])
