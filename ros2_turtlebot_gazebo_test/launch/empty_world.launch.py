# Copyright 2018 Tamaki Nishino
#
"""Launch a Gazebo server with an empty worlds."""

import asyncio
import logging
import os
import re
import time

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros
import rclpy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity

_logger = logging.getLogger('empty_world.launch')

class Spawn(launch.action.Action):
    node = None

    def __init__(self, name, xml, initial_pose=None, robot_namespace="", **kwargs):
        super().__init__(**kwargs)
        self.__name = name
        self.__xml = xml
        self.__robot_namespace = robot_namespace
        if initial_pose is None:
            self.__initial_pose = Pose()
            self.__initial_pose.orientation.w = 1.0
        else:
            self.__initial_pose = initial_pose
        self.__completed_future = None

    async def _wait_and_spawn(self, context):
        try:
            node = rclpy.create_node('Spawner')
            rclpy.get_global_executor().add_node(node)
            cli = node.create_client(SpawnEntity, 'spawn_entity')
            while True:
                if cli.service_is_ready():
                    res = await cli.call_async(SpawnEntity.Request(
                        name=self.__name,
                        xml=self.__xml,
                        robot_namespace=self.__robot_namespace,
                        initial_pose=self.__initial_pose))
                    if not res.success:
                        _logger.error("failed to spawn: {0}".format(res.status_message))
                    else:
                        _logger.info("spawned: {0}".format(res.status_message))
                    break
                await asyncio.sleep(1)
        finally:
            node.destroy_client(cli)
            node.destroy_node()

        self.__completed_future.set_result(None)

    def execute(self, context):
        self.__completed_future = context.asyncio_loop.create_future()
        context.asyncio_loop.create_task(self._wait_and_spawn(context))
        return None

    def get_asyncio_future(self):
        return self.__completed_future
    
def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('turtlebot_description'), 'urdf', 'create_circles_kinect.urdf')

    gazebo = launch.actions.ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen',
    )
    namespace = 'tb1'
    teleop_twist_joy = launch_ros.actions.Node(
        package='teleop_twist_joy',
        node_executable='teleop_node',
        remappings=[('/cmd_vel', '/{0}/cmd_vel'.format(namespace))])

    joy = launch_ros.actions.Node(
        package='joy',
        node_executable='joy_node',
        node_namespace=namespace)

    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        arguments=[urdf],
        node_namespace=namespace)

    xml = open(urdf, 'r').read()
    def replace_package_path(match):
        f = os.path.join('file://' + get_package_share_directory(match[1]) + '/')
        return f
    xml = re.sub(r'package\://([^/]*)/', replace_package_path, xml)
    p1 = Pose()
    p1.position.x = 0.0
    p1.orientation.w = 1.0
    spawn_robot = Spawn('turtlebot1', xml, initial_pose=p1, robot_namespace=namespace)

    return launch.LaunchDescription([
        gazebo,
        robot_state_publisher,
        teleop_twist_joy,
        joy,
        spawn_robot
    ])
