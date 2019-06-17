#!/usr/bin/python
"""
    file: publish_one_image.launch.py
    purpose: Launch the image publisher node.
"""

import launch
import launch_ros.actions


def generate_launch_description():
    publisher = launch_ros.actions.Node(
        package='image_capture', node_executable='image_publisher', output='screen')
    return launch.LaunchDescription([
        publisher,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=publisher,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )),
    ])
