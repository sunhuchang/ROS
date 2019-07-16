# Copyright 2019 sunhuchang.
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
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   # TODO(wjwwood): Use a substitution to find share directory once this is implemented in launch
   
   return LaunchDescription([
       Node(package='LH_laser_driver', node_executable='LH_laser_publisher',
            output='screen'),
       Node(package='cartographer_ros', node_executable='cartographer_node', output='screen',arguments=["-configuration_directory","/home/orion-jetsonnano/sunhuchang/ros2/cartographer_ros/cartographer_ros/configuration_files","-configuration_basename","2dLidar_mapping.lua"]),
       Node(package='cartographer_ros', node_executable='occupancy_grid_node',
            output='screen', arguments=["-resolution","0.05"])
])
