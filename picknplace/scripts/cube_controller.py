#!/usr/bin/env python3
#
# Copyright 2019 Open Source Robotics Foundation
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
# Desc: helper script for spawning entities in gazebo
# Author: John Hsu, Dave Coleman
#
import argparse
import math
import os
import sys
from urllib.parse import SplitResult, urlsplit
from xml.etree import ElementTree

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, GetEntityState
from robot_interface.msg import BoxState
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Empty
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from ament_index_python.packages import get_package_share_directory
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters
import robot_config.utils as utils

import time
import random
from copy import copy, deepcopy

DEFAULT_TIMEOUT = 30.0
class CubeController(Node):
    # Node to spawn an entity in Gazebo.
    def __init__(self, args):
        super().__init__('Cube_Controller')
        self.last_spawn = 0

    def run(self):
        """
        Run node, spawning entity and doing other actions as configured in program arguments.

        Returns exit code, 1 for failure, 0 for success
        """
        # Wait for entity to exist if wait flag is enabled
        picknplace = get_package_share_directory('picknplace')
        # f = open(robot_config + '/urdf/objects/box.urdf', 'r')
        f = open(picknplace + '/urdf/marker_0/model.sdf')
        self.entity_xml = f.read()
        # Form requested Pose from arguments
        self.initial_pose = Pose()
        self.initial_pose.position.x = float(0.6)
        self.initial_pose.position.y = float(-3.0)  # float(-3.7)
        self.initial_pose.position.z = float(0.5)
        q = utils.quaternion_from_euler(0.0, 0.0, 0.0)
        self.initial_pose.orientation.w = q[0]
        self.initial_pose.orientation.x = q[1]
        self.initial_pose.orientation.y = q[2]
        self.initial_pose.orientation.z = q[3]
        self.callback_group = ReentrantCallbackGroup()
        self.client_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.index = 0
        self.cubes = [None for i in range(5)]
        self.arm1_range = [0.25, -3.5, 1.5, -1.7]
        self.arm2_range = [0.25, -1.7, 1.5, -.2]
        self.delete_range = [0.25, .1, 1.5, 2.0]
        self.turtlebot_range = [-0.4, -0.4, 0.0, -.1]

        self.spawn_client = self.create_client(
            SpawnEntity, '/spawn_entity', callback_group=self.client_cb_group)
        if not self.spawn_client.wait_for_service(timeout_sec=DEFAULT_TIMEOUT):
            self.get_logger().error(
                'Service %s/spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?')

        self.delete_client = self.create_client(
            DeleteEntity, '/delete_entity', callback_group=self.client_cb_group)
        if not self.delete_client.wait_for_service(timeout_sec=DEFAULT_TIMEOUT):
            self.get_logger().error(
                'Service %s/spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?')

        self.entity_state_client = self.create_client(
            GetEntityState, '/get_entity_state', callback_group=self.client_cb_group)
        if not self.entity_state_client.wait_for_service(timeout_sec=DEFAULT_TIMEOUT):
            self.get_logger().error(
                'Service %s/get_entity_state unavailable. Was Gazebo started with GazeboRosFactory?')


        location_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.object_location_publisher0 = self.create_publisher(BoxState, '/object_location0', qos_profile=location_qos_profile, callback_group=self.client_cb_group)
        
        self.object_location_publisher1 = self.create_publisher(
            BoxState, '/object_location1', qos_profile=location_qos_profile, callback_group=self.client_cb_group)
        
        self.trigger_turtlebot_nav = self.create_publisher(
            String, '/trigger_move', qos_profile=location_qos_profile, callback_group=self.client_cb_group)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Clock, '/clock', self.clock_cb, callback_group=self.callback_group, qos_profile=qos_profile)

        self.timer = self.create_timer(
            1, self.timer_callback, callback_group=self.timer_cb_group)

        return 0

    def timer_callback(self):
        self.timer.cancel()
        start = time.time()
        array_len = len(self.cubes)
        cubelist = ''
        try:
            for index in range(array_len):
                if self.cubes[index] is not None:
                    cube = self.cubes[index]
                    pose = self._cube_location(cube)
                    if pose.position.z < .1 or utils.pointInRect(pose, self.delete_range):  # block is one the floor so remove it
                        self._delete_cube(index)
                        array_len-=1
                    elif utils.pointInRect(pose, self.arm1_range):
                        self.object_location_publisher1.publish(pose)
                    elif utils.pointInRect(pose, self.arm2_range):
                        box_state = BoxState()
                        box_state.name =cube
                        box_state.pose=pose
                        self.object_location_publisher0.publish(box_state)
                        # elif pointInRect(pose, self.turtlebot_range) and pose.position.z < 0.21 and pose.position.z > 0.18:
                    else:
                        pass
                        #pos = utils.entity_location(self, "turtlebot3", cube)
                        #if pos.position.x < .2 and pos.position.x > -.2 and pos.position.y < .2 and pos.position.y > -.2 and pose.position.z < 0.21 and pose.position.z > 0.18:
                        #    cubelist += str(index) + ','
                            # msg = String()
                            # msg.data = deepcopy(cube)
                            # self.get_logger().info(f"**** object on turtlebot .. triggering move")
                            # self.trigger_turtlebot_nav.publish(msg)
                            # call_set_parameters(self, "Turtlebot3Controller", Parameter('goto_arm', Parameter.Type.INTEGER, 1))
                            # call_set_parameters(self, "Turtlebot3Controller", Parameter('cube_name', Parameter.Type.STRING, cube))
                        #    pass
        except Exception as e:
            pass
    
        if cubelist != '':
            utils.call_set_parameters(self, "Turtlebot3Controller", Parameter(
                'goto_arm', Parameter.Type.INTEGER, 1))
            utils.call_set_parameters(self, "Turtlebot3Controller", Parameter(
                'cube_name', Parameter.Type.STRING, cubelist))

        self.timer.reset()

    def is_cube_missed(self, pose):
        if pose.x > 0.0 and pose.x < 1.5 and pose.y > 1.7 and pose.y < 3.0:
            return True
        else:
            return False

    def clock_cb(self, entity):
        
        if entity.clock.sec - self.last_spawn > 10 or self.last_spawn == 0:
            
            self.last_spawn = entity.clock.sec
            array_len = len(self.cubes)
            for index in range(array_len):
                if self.cubes[index] == None:
                    success = self._spawn_cube(index)
                    return

    def _spawn_cube(self, index):
        if self.spawn_client is None:
            self.get_logger().error(
                'Service %s/spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?')
            return False
        print("Spawning cube")
        req = SpawnEntity.Request()
        req.name = 'cube_' + str(index)
        req.xml = self.entity_xml
        req.robot_namespace = ''
        req.initial_pose = deepcopy(self.initial_pose)
        req.initial_pose.position.x += random.uniform(-0.15, 0.15)
        req.initial_pose.position.y += random.uniform(-0.15, 0.15)
        req.reference_frame = ''
        srv_call = self.spawn_client.call(req)
        self.cubes[index] = req.name
        return 0

    def _cube_location(self, cube_name):
        if self.entity_state_client is None:
            self.get_logger().error(
                'Service %s/get_entity_state unavailable. Was Gazebo started with GazeboRosFactory?')
            return False

        req = GetEntityState.Request()
        req.name = cube_name
        req.reference_frame = 'world'

        srv_call = self.entity_state_client.call(req)
        if srv_call.success == False:
            self.get_logger().error(f'{cube_name} not found')
            self.cubes.remove(cube_name)
        return srv_call.state.pose

    # TODO(shivesh): Wait for https://github.com/ros2/rclpy/issues/244
    def _delete_cube(self, index):
        # Delete entity from gazebo on shutdown if bond flag enabled
        if self.delete_client is None:
            self.get_logger().error(
                'Service %s/delete_entity unavailable. Was Gazebo started with GazeboRosFactory?')
            return False

        req = DeleteEntity.Request()
        req.name = 'cube_' + str(index)

        srv_call = self.delete_client.call(req)
        self.cubes[index] = None

        return 0

def main(args=sys.argv):
    rclpy.init(args=args)
    args_without_ros = rclpy.utilities.remove_ros_args(args)
    cube_controller_node = CubeController(args_without_ros)
    cube_controller_node.get_logger().info('Spawn Entity started')

    executor = MultiThreadedExecutor()
    executor.add_node(cube_controller_node)
    exit_code = cube_controller_node.run()
    executor.spin()

    sys.exit(0)


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        import traceback
        traceback.print_exc()
