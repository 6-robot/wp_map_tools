#!/usr/bin/env python3
#
# Copyright 2023 6-robot.
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
#
# Authors: Zhang Wanjie

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.actions
import launch.actions
import launch.events

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('wp_map_tools'), 'launch')

    # 声明launch文件参数，用于指定map.yaml文件路径
    map_file = os.path.join(
        get_package_share_directory('wpb_home_tutorials'),
        'maps',
        'map.yaml'
    )

    # 创建map_server节点
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': True}]
    )

    lifecycle_nodes = ['map_server']
    use_sim_time = True
    autostart = True

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    map_tf_cmd = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'])

    wp_edit_cmd = Node(
            package='wp_map_tools',
            executable='wp_edit_node',
            name='wp_edit_node',
            output='screen'
        )
    
    rviz_cmd = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(get_package_share_directory('wp_map_tools'), 'rviz', 'edit.rviz')]]
        )

    # 创建一个事件处理程序，用于在Rviz2退出时发送一个事件
    shutdown_handler = launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessExit(
            target_action=rviz_cmd,
            on_exit=[
                launch.actions.LogInfo(
                    msg=(launch.substitutions.EnvironmentVariable(name='USER'),' closed the RViz2 window')
                    ),
                launch.actions.EmitEvent(event=launch.events.Shutdown(reason='Window closed'))
            ]
        )
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(map_tf_cmd)
    ld.add_action(wp_edit_cmd)
    ld.add_action(rviz_cmd)
    # 添加事件处理程序到LaunchDescription
    ld.add_action(shutdown_handler)

    return ld
