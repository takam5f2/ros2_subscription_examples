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

import launch
import launch_ros.actions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    use_intra_process_comms_value = launch.substitutions.LaunchConfiguration("use_intra_process_comms", default="true")
    use_intra_process_comms_argument = launch.actions.DeclareLaunchArgument("use_intra_process_comms", default_value="true", description="Whether to use intra-process comms")
    # use transient local
    use_transient_local_value = launch.substitutions.LaunchConfiguration("use_transient_local", default="false")
    use_transient_local_argument = launch.actions.DeclareLaunchArgument("use_transient_local", default_value="false", description="Whether to use transient local durability")
    # frequency
    talker_update_frequency_value = launch.substitutions.LaunchConfiguration("talker_update_frequency", default=1.0)
    talker_update_frequency_argument = launch.actions.DeclareLaunchArgument("talker_update_frequency", default_value='1.0', description="Update frequency per seconds for talker")
    listener_update_frequency_value = launch.substitutions.LaunchConfiguration("listener_update_frequency", default=1.0)
    listener_update_frequency_argument = launch.actions.DeclareLaunchArgument("listener_update_frequency", default_value='1.0', description="Update frequency per seconds for listener")


    container = ComposableNodeContainer(
        name = 'intra_process_talker_listener',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='intra_process_talker_listener', 
                plugin='intra_process_talker_listener::TalkerIntraProcess',
                name='talker_intra_process',
                namespace='',
                parameters=[{'use_transient_local': use_transient_local_value,
                'update_frequency': talker_update_frequency_value,
                }],
                extra_arguments=[{'use_intra_process_comms': use_intra_process_comms_value}]
            ),
            ComposableNode(
                package='intra_process_talker_listener', 
                plugin='intra_process_talker_listener::TimerListenerIntraProcess',
                name='timer_listener_intra_process',
                namespace='',
                parameters=[{'use_transient_local': use_transient_local_value,
                'update_frequency': listener_update_frequency_value,
                }],
                extra_arguments=[{'use_intra_process_comms': use_intra_process_comms_value}]
            )
        ],
    )

    return launch.LaunchDescription([
        use_intra_process_comms_argument,
        use_transient_local_argument,
        talker_update_frequency_argument,
        listener_update_frequency_argument,
        container
    ])
