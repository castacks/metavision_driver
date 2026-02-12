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
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


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
    nodes = [
        ComposableNode(
        package="rosbag2_composable_recorder",
        plugin="rosbag2_composable_recorder::ComposableRecorder",
        name="event_recorder",
        parameters=[
            {
                "topics": ["/" + cam_name + "/events" for cam_name in cameras] + ["/zed/zed_node/left/image_rect_color","/zed/zed_node/right/image_rect_color"],
                "bag_name": LaunchConfig("bag"),
                "bag_prefix": LaunchConfig("bag_prefix"),
            },
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
        )
    ]
    return nodes

def make_separate_recorders(cameras, bag_name, bag_prefix):
    nodes = [
        ComposableNode(
            package="rosbag2_composable_recorder",
            plugin="rosbag2_composable_recorder::ComposableRecorder",
            name=cam_name + "_recorder",
            parameters=[
                {
                    "topics": ["/" + cam_name + "/events"],
                    "bag_name": bag_name,
                    "bag_prefix": bag_prefix+"_"+cam_name+"_",
                },
            ],
            remappings=[("/start_recording", cam_name + "/start_recording"), 
                         ("/stop_recording", cam_name + "/stop_recording")],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
        for cam_name in cameras
    ]
    return nodes

def launch_setup(context, *args, **kwargs):
    """Create composable node."""
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
    nodes = make_cameras(cameras, specific_params)
    if IfCondition(LaunchConfig("with_renderer")).evaluate(context):
        nodes += make_renderers(cameras)
    if IfCondition(LaunchConfig("with_fibar")).evaluate(context):
        nodes += make_fibars(cameras)
    if IfCondition(LaunchConfig("with_recorder")).evaluate(context):
        nodes += make_recorders(cameras)
    if IfCondition(LaunchConfig("with_separate_recorders")).evaluate(context):
        nodes += make_separate_recorders(cameras, bag_name, bag_prefix)
    container = ComposableNodeContainer(
        name="metavision_driver_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_isolated",
        composable_node_descriptions=nodes,
        output="screen",
    )
    debug_with_libasan = False
    if debug_with_libasan:
        preload = SetEnvironmentVariable(
            name="LD_PRELOAD", value="/usr/lib/gcc/x86_64-linux-gnu/13/libasan.so"
        )
        asan_options = SetEnvironmentVariable(
            name="ASAN_OPTIONS", value="new_delete_type_mismatch=0"
        )
        return [preload, asan_options, container]
    return [container]


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
                default_value=["fps"],
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
            LaunchArg(
                "with_separate_recorders",
                default_value="false",
                description="if separate recorders should be started for each camera",
            ),
            LaunchArg(
                "with_recorder",
                default_value="false",
                description="if a combined event recorder should be started for all cameras",
            ),
            LaunchArg("bag", default_value=[""], description="name of output bag"),
            LaunchArg("bag_prefix", default_value=["stereo_events_"], description="prefix of output bag"),
            OpaqueFunction(function=launch_setup),
        ]
    )
