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
"""
Per-camera containers for Metavision stereo (event cameras).

Changes vs stereo_driver.launch.py:
- Each camera gets its own ComposableNodeContainer holding:
    - metavision_driver::DriverROS2
    - optional rosbag2_composable_recorder::ComposableRecorder (per camera)
    - optional renderer / fibar (per camera)
- No single container for both cameras.
- Added a launch option to run cameras either:
    - "sync" (camera_0=primary, camera_1=secondary, with ready-topic wiring)
    - "standalone" (both cameras use sync_mode="standalone")
"""

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
    "trigger_in_mode": "external",
}


def make_driver_node(cam_name: str, serial: str, settings: str, sync_mode: str, remappings):
    return ComposableNode(
        package="metavision_driver",
        plugin="metavision_driver::DriverROS2",
        name=cam_name,
        parameters=[
            common_params,
            {"serial": serial},
            {"settings": settings},
            {"sync_mode": sync_mode},
        ],
        remappings=remappings,
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def make_renderer_node(cam_name: str, fps: float):
    return ComposableNode(
        package="event_camera_renderer",
        plugin="event_camera_renderer::Renderer",
        name=cam_name + "_renderer",
        parameters=[{"fps": fps}],
        remappings=[("~/events", cam_name + "/events")],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def make_fibar_node(cam_name: str, fps: float):
    return ComposableNode(
        package="event_image_reconstruction_fibar",
        plugin="event_image_reconstruction_fibar::Fibar",
        name=cam_name + "_fibar",
        parameters=[{"fps": fps}],
        remappings=[("~/events", cam_name + "/events")],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def make_recorder_node(cam_name: str, bag_name: str, bag_prefix: str):
    # Record only this camera's /<cam>/events
    # Note: ComposableRecorder exposes /start_recording and /stop_recording services.
    # We remap those to /<cam>/start_recording and /<cam>/stop_recording to avoid collisions.
    return ComposableNode(
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
            ("/stop_recording", cam_name + "/stop_recording"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def make_camera_container(container_name: str, nodes):
    return ComposableNodeContainer(
        name=container_name,
        namespace="",
        package="rclcpp_components",
        executable="component_container_isolated",
        composable_node_descriptions=nodes,
        output="screen",
    )


def launch_setup(context, *args, **kwargs):
    cam_0 = LaunchConfig("camera_0_name").perform(context)
    cam_1 = LaunchConfig("camera_1_name").perform(context)

    cam_0_serial = LaunchConfig("camera_0_serial").perform(context)
    cam_1_serial = LaunchConfig("camera_1_serial").perform(context)

    cam_0_settings = LaunchConfig("camera_0_settings").perform(context)
    cam_1_settings = LaunchConfig("camera_1_settings").perform(context)

    # recorder params
    bag_name = LaunchConfig("bag").perform(context)
    bag_prefix = LaunchConfig("bag_prefix").perform(context)

    # per-camera visualization FPS (renderer/fibar)
    fps = float(LaunchConfig("fps").perform(context))

    # sync option: "sync" or "standalone"
    sync_option = LaunchConfig("sync_option").perform(context).strip().lower()
    if sync_option == "standalone":
        cam_0_sync_mode = "standalone"
        cam_1_sync_mode = "standalone"
        cam_0_remaps = [("~/events", cam_0 + "/events")]
        cam_1_remaps = [("~/events", cam_1 + "/events")]
    else:
        # default to sync behavior (primary/secondary)
        cam_0_sync_mode = "primary"
        cam_1_sync_mode = "secondary"
        # Primary publishes its ~/ready onto /<cam_1>/ready so secondary can consume it
        cam_0_remaps = [
            ("~/events", cam_0 + "/events"),
            ("~/ready", cam_1 + "/ready"),
        ]
        cam_1_remaps = [("~/events", cam_1 + "/events")]

    # Build per-camera node lists
    cam0_nodes = [make_driver_node(cam_0, cam_0_serial, cam_0_settings, cam_0_sync_mode, cam_0_remaps)]
    cam1_nodes = [make_driver_node(cam_1, cam_1_serial, cam_1_settings, cam_1_sync_mode, cam_1_remaps)]

    if IfCondition(LaunchConfig("with_renderer")).evaluate(context):
        cam0_nodes.append(make_renderer_node(cam_0, fps))
        cam1_nodes.append(make_renderer_node(cam_1, fps))

    if IfCondition(LaunchConfig("with_fibar")).evaluate(context):
        cam0_nodes.append(make_fibar_node(cam_0, fps))
        cam1_nodes.append(make_fibar_node(cam_1, fps))

    if IfCondition(LaunchConfig("with_recorder")).evaluate(context):
        cam0_nodes.append(make_recorder_node(cam_0, bag_name, bag_prefix))
        cam1_nodes.append(make_recorder_node(cam_1, bag_name, bag_prefix))

    # Separate containers (one per camera)
    cam0_container = make_camera_container("metavision_" + cam_0 + "_container", cam0_nodes)
    cam1_container = make_camera_container("metavision_" + cam_1 + "_container", cam1_nodes)

    debug_with_libasan = False
    if debug_with_libasan:
        preload = SetEnvironmentVariable(
            name="LD_PRELOAD", value="/usr/lib/gcc/x86_64-linux-gnu/13/libasan.so"
        )
        asan_options = SetEnvironmentVariable(
            name="ASAN_OPTIONS", value="new_delete_type_mismatch=0"
        )
        return [preload, asan_options, cam0_container, cam1_container]

    return [cam0_container, cam1_container]


def generate_launch_description():
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
                default_value=["25.0"],
                description="renderer / fibar frame rate in Hz",
            ),
            LaunchArg(
                "with_renderer",
                default_value="false",
                description="if renderers should be started as well (per camera)",
            ),
            LaunchArg(
                "with_fibar",
                default_value="false",
                description="if fibar reconstruction should be started as well (per camera)",
            ),
            LaunchArg(
                "with_recorder",
                default_value="false",
                description="if per-camera event recorders should be started",
            ),
            LaunchArg(
                "sync_option",
                default_value="sync",
                description='Camera sync mode: "sync" (primary+secondary) or "standalone" (both standalone)',
            ),
            LaunchArg("bag", default_value=[""], description="name of output bag"),
            LaunchArg(
                "bag_prefix",
                default_value=["stereo_events"],
                description="prefix of output bag (camera name is appended automatically)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
