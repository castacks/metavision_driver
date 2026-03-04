# -----------------------------------------------------------------------------
# Copyright 2025 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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
#

import launch
import os
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import sys
sys.path.insert(0, get_package_share_directory("ros_rec") + "/launch")
from composable import make_recorder_nodes  # type: ignore


common_params = {
    "use_multithreading": False,
    "bias_file": "",
    "camerainfo_url": "",
    "event_message_time_threshold": 1.0e-3,
    "trigger_in_mode": "external"
}


def make_renderers(cameras):
    nodes = [
        ComposableNode(
            package="event_camera_renderer",
            plugin="event_camera_renderer::Renderer",
            name=cam + "_renderer",
            parameters=[{"fps": 25.0}],
            remappings=[("~/events", cam + "/events")],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
        for cam in cameras
    ]
    return nodes


def make_fibars(cameras):
    nodes = [
        ComposableNode(
            package="event_image_reconstruction_fibar",
            plugin="event_image_reconstruction_fibar::Fibar",
            name=cam + "_fibar",
            parameters=[{"fps": 25.0}],
            remappings=[("~/events", cam + "/events")],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
        for cam in cameras
    ]
    return nodes


def make_cameras(cameras, params):
    nodes = [
        ComposableNode(
            package="metavision_driver",
            plugin="metavision_driver::DriverROS2",
            name=cam,
            parameters=[
                common_params,
                {"serial": params[cam]["serial"]},
                {"settings": params[cam]["settings"]},
                {"sync_mode": params[cam]["sync_mode"]},
            ],
            remappings=params[cam]["remappings"],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
        for cam in cameras
    ]
    return nodes


def make_recorders(cameras):
    """Create combined rosbag2 composable recorder"""
    nodes = [
        ComposableNode(
            package="rosbag2_composable_recorder",
            plugin="rosbag2_composable_recorder::ComposableRecorder",
            name="event_recorder",
            parameters=[
                {
                    "topics": ["/" + cam_name + "/events" for cam_name in cameras],
                    "bag_name": LaunchConfig("bag"),
                    "bag_prefix": LaunchConfig("bag_prefix"),
                },
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    ]
    return nodes


def make_separate_recorders(cameras, bag_name, bag_prefix):
    """Create separate rosbag2 composable recorders for each camera"""
    nodes = [
        ComposableNode(
            package="rosbag2_composable_recorder",
            plugin="rosbag2_composable_recorder::ComposableRecorder",
            name=cam_name + "_recorder",
            parameters=[
                {
                    "topics": ["/" + cam_name + "/events"],
                    "bag_name": bag_name,
                    "bag_prefix": bag_prefix + "_" + cam_name + "_",
                },
            ],
            remappings=[
                ("/start_recording", cam_name + "/start_recording"),
                ("/stop_recording", cam_name + "/stop_recording")
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
        for cam_name in cameras
    ]
    return nodes


def launch_setup(context, *args, **kwargs):
    """Create composable node container and optional separate nodes."""
    cam_0_name = LaunchConfig("camera_0_name")
    cam_0 = cam_0_name.perform(context)
    cam_1_name = LaunchConfig("camera_1_name")
    cam_1 = cam_1_name.perform(context)
    cameras = (cam_0, cam_1)
    bag_name = LaunchConfig("bag").perform(context)
    bag_prefix = LaunchConfig("bag_prefix").perform(context)
    
    specific_params = {
        cam_0: {
            "serial": LaunchConfig("camera_0_serial").perform(context),
            "settings": LaunchConfig("camera_0_settings").perform(context),
            "sync_mode": "primary",
            "remappings": [
                ("~/events", cam_0 + "/events"),
                ("~/ready", cam_1 + "/ready"),
            ],
        },
        cam_1: {
            "serial": LaunchConfig("camera_1_serial").perform(context),
            "settings": LaunchConfig("camera_1_settings").perform(context),
            "sync_mode": "secondary",
            "remappings": [("~/events", cam_1 + "/events")],
        },
    }
    
    # Build composable nodes list
    nodes = make_cameras(cameras, specific_params)
    node_list_0 = [nodes[0]]  # First camera in container 0
    node_list_1 = [nodes[1]]  # Second camera in container 1
    
    if IfCondition(LaunchConfig("with_renderer")).evaluate(context):
        nodes += make_renderers(cameras)
    
    if IfCondition(LaunchConfig("with_fibar")).evaluate(context):
        nodes += make_fibars(cameras)
    
    # Recorder type selection
    recorder_type = LaunchConfig("recorder_type").perform(context)
    
    if recorder_type == "combined":
        nodes += make_recorders(cameras)
    elif recorder_type == "separate":
        # nodes += make_separate_recorders(cameras, bag_name, bag_prefix)
        node_list_0 += make_recorder_nodes(keys=[f"event_left"])
        node_list_1 += make_recorder_nodes(keys=[f"event_right"])

    # bag_record_pid is launched as a separate node below
    
    # Create the composable node container
    container_0 = ComposableNodeContainer(
        name="metavision_driver_container_0",
        namespace="",
        package="rclcpp_components",
        executable="component_container_isolated",
        composable_node_descriptions=node_list_0,
        output="screen",
    )

    container_1 = ComposableNodeContainer(
        name="metavision_driver_container_1",
        namespace="",
        package="rclcpp_components",
        executable="component_container_isolated",
        composable_node_descriptions=node_list_1,
        output="screen",
    )

    
    
    launch_list = [container_0, container_1]
    
    # Add bag_record_pid as a separate Python node if requested
    if recorder_type == "bag_record_pid":
        try:
            pkg_dir = get_package_share_directory('bag_record_pid')
        except:
            pkg_dir = ''
        
        cfg_path = LaunchConfig("bag_record_pid_cfg").perform(context)
        if not cfg_path and pkg_dir:
            cfg_path = os.path.join(pkg_dir, 'config', 'tartan_rgbt.yaml')
        
        output_dir = LaunchConfig("bag_record_pid_output_dir").perform(context)
        if not output_dir:
            output_dir = '/logging'
        
        mcap_qos_dir = LaunchConfig("bag_record_pid_mcap_qos_dir").perform(context)
        if not mcap_qos_dir and pkg_dir:
            mcap_qos_dir = os.path.join(pkg_dir, 'config')
        
        best_effort_qos_str = LaunchConfig("bag_record_pid_best_effort_qos").perform(context)
        best_effort_qos = best_effort_qos_str.lower() in ['true', '1', 'yes']
        
        bag_recorder_node = Node(
            package='bag_record_pid',
            executable='bag_record_node',
            name='bag_record_pid',
            parameters=[{
                'cfg_path': cfg_path,
                'output_dir': output_dir,
                'mcap_qos_dir': mcap_qos_dir,
                'best_effort_qos_sub': best_effort_qos
            }],
            output='screen'
        )
        
        launch_list.append(bag_recorder_node)
    
    # Debug with libasan if needed
    debug_with_libasan = False
    if debug_with_libasan:
        preload = SetEnvironmentVariable(
            name="LD_PRELOAD", value="/usr/lib/gcc/x86_64-linux-gnu/13/libasan.so"
        )
        asan_options = SetEnvironmentVariable(
            name="ASAN_OPTIONS", value="new_delete_type_mismatch=0"
        )
        return [preload, asan_options] + launch_list
    
    return launch_list


def generate_launch_description():
    """Create composable node by calling opaque function."""
    return launch.LaunchDescription(
        [
            LaunchArg(
                "camera_0_name",
                default_value=["event_cam_0"],
                description="camera name of camera 0",
            ),
            LaunchArg(
                "camera_1_name",
                default_value=["event_cam_1"],
                description="camera name of camera 1",
            ),
            LaunchArg(
                "camera_0_serial",
                default_value=["4110030785"],
                description="serial number of camera 0",
            ),
            LaunchArg(
                "camera_1_serial",
                default_value=["4110030791"],
                description="serial number of camera 1",
            ),
            LaunchArg(
                "camera_0_settings",
                default_value=[""],
                description="settings file for camera 0",
            ),
            LaunchArg(
                "camera_1_settings",
                default_value=[""],
                description="settings file for camera 1",
            ),
            LaunchArg(
                "fps",
                default_value=["25"],
                description="renderer frame rate in Hz",
            ),
            LaunchArg(
                "with_renderer",
                default_value="false",
                description="if renderers should be started as well",
            ),
            LaunchArg(
                "with_fibar",
                default_value="false",
                description="if fibar reconstruction should be started as well",
            ),
            # New unified recorder type argument
            LaunchArg(
                "recorder_type",
                default_value="none",
                description="Type of recorder: 'none', 'combined', 'separate', or 'bag_record_pid'",
            ),
            # Legacy arguments for backward compatibility (deprecated)
            LaunchArg(
                "with_separate_recorders",
                default_value="false",
                description="[DEPRECATED] Use recorder_type='separate' instead",
            ),
            LaunchArg(
                "with_recorder",
                default_value="false",
                description="[DEPRECATED] Use recorder_type='combined' instead",
            ),
            # bag_record_pid specific arguments
            LaunchArg(
                "bag_record_pid_cfg",
                default_value="",
                description="Config file path for bag_record_pid (empty = use default)",
            ),
            LaunchArg(
                "bag_record_pid_output_dir",
                default_value="",
                description="Output directory for bag_record_pid (empty = /logging)",
            ),
            LaunchArg(
                "bag_record_pid_mcap_qos_dir",
                default_value="",
                description="MCAP QoS directory for bag_record_pid (empty = use default)",
            ),
            LaunchArg(
                "bag_record_pid_best_effort_qos",
                default_value="true",
                description="Best effort QoS for bag_record_pid subscriber",
            ),
            # General bag arguments
            LaunchArg("bag", default_value=[""], description="name of output bag"),
            LaunchArg("bag_prefix", default_value=["stereo_events_"], description="prefix of output bag"),
            OpaqueFunction(function=launch_setup),
        ]
    )