#!/usr/bin/env python3
#

import time
import threading
import sys
from copy import deepcopy

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from pymoveit2 import MoveIt2
from smach import State, StateMachine
from smach_ros import RosState

from robot_config.utils import *
from robot_interface.msg import BoxState
from pymoveit2.robots import ur5 as robot 
from picknplace.basic_navigator import BasicNavigator

class RobotController(Node):
        # Node to spawn an entity in Gazebo.
        def __init__(self, args):
                super().__init__('ARM1Controller')
                qos_profile = QoSProfile(
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=1
                )

                self.callback_group0 = MutuallyExclusiveCallbackGroup()
                self.gripper_group0 = MutuallyExclusiveCallbackGroup()
                self.callback_group1 = MutuallyExclusiveCallbackGroup()
                self.client_cb_group0 = MutuallyExclusiveCallbackGroup()
                self.client_cb_group1 = MutuallyExclusiveCallbackGroup()
                self.timer_cb_group = MutuallyExclusiveCallbackGroup()
                self.moveit_callback_group0 = MutuallyExclusiveCallbackGroup()
                self.moveit_callback_group1 = MutuallyExclusiveCallbackGroup()

                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self)

                time.sleep(1)

                self.moveit2_robot0 = MoveIt2(
                    node=self,
                    joint_names=robot.joint_names(),
                    base_link_name=robot.base_link_name(),
                    end_effector_name=robot.end_effector_name(),
                    group_name=robot.MOVE_GROUP_ARM,
                    callback_group=self.moveit_callback_group0,
                    execute_via_moveit=False,
                    ignore_new_calls_while_executing=True,
                    namespace_prefix='/arm1/'
                )

                self.navigator = BasicNavigator()

                self.logger = self.get_logger()
                self.logger.set_level(LoggingSeverity.INFO)
                # self.moveit2_robot0.set_path_constraint(position=[3.,3.,3.])
                self.grasping = False
                # self.transform_to_world = self.tf_buffer.lookup_transform(
                #                'base_link','world', rclpy.time.Time())

                #outcome = sm.execute()
                self.execute_state_machine=False
                self.robot_subscription0 = self.create_subscription(
                    BoxState, '/object_location0', self.object_location0_cb, callback_group=self.callback_group0, qos_profile=qos_profile)

                self.robot_subscription1 = self.create_subscription(
                    BoxState, '/object_location1', self.object_location1_cb, callback_group=self.callback_group1, qos_profile=qos_profile)

                self.robot_subscription2 = self.create_subscription(
                    Bool, '/arm1/grasping', self.object_grasping, callback_group=self.client_cb_group0, qos_profile=qos_profile)


                self.gripper_service_ = self.create_client(
                    SetBool, "/arm1/switch", callback_group=self.gripper_group0)

                if not self.gripper_service_.service_is_ready():
                    self.gripper_service_.wait_for_service()
                    self.get_logger().info("...connected!")

                #self.timer = self.create_timer(.1, self.state_machine_callback,
                #              callback_group=self.timer_cb_group)

        def state_machine_callback(self):
                self.timer.cancel()
                if self.execute_state_machine==True:
                        self.get_logger().info(f"\n\n\nStarting state machine")
                        result=self.sm.execute()

                        self.get_logger().info(f"Execution results **** {result}")
                        self.execute_state_machine=False

                self.timer.reset()

        def object_location0_cb(self, pose):
            #self.get_logger().info(f"cube location {pose.position.x - 0.20}, {pose.position.y}")
            self.last_pose = deepcopy(pose)
            '''
            if  self.execute_state_machine==False: # and pose.position.x != 0 and pose.position.y < -0.1 and pose.position.x > .5 and pose.position.x < 1.5:
                self.last_pose = deepcopy(pose)
                self.logger.info(f"*** Move action status")
                self.execute_state_machine=True
            '''
            pass

        def object_location1_cb(self, pose):
            pass

        def object_grasping(self, msg):
            self.grasping=msg.data

        def gripper_on(self):
            self.grasping=False
            request = SetBool.Request()
            request.data = True
            self.gripper_service_.call(request)

        def gripper_off(self):
            
            request = SetBool.Request()
            request.data = False
            self.gripper_service_.call(request)

        def amr_goto_pose(self, x, y, z):
            # Go to our demos first goal pose
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y
            goal_pose.pose.orientation.z = z
            goal_pose.pose.orientation.w = 1.0
            self.navigator.goToPose(goal_pose)

class Setup(RosState):
    def __init__(self, robot_controller):
        State.__init__(self, outcomes=['home','failed'])
        self.robot_controller=robot_controller

    def execute(self, userdata):
        self.robot_controller.gripper_off()
        intermediat_stage = [0.5, 0.5, 0.011]        
        self.robot_controller.moveit2_robot0.move_to_pose(position=intermediat_stage, quat_xyzw=[
                                                1.0, 0.0, 0.0, 0.0], cartesian=True, frame_id='world')
        self.robot_controller.moveit2_robot0.wait_until_executed()
        return 'home'
             
class Home(RosState):
    def __init__(self, robot_controller):
        State.__init__(self, outcomes=['wait','failed'])
        self.robot_controller=robot_controller

    def execute(self, userdata):
        self.robot_controller.gripper_off()
        home_pos = [0.5, -0.5, 0.011]
        self.robot_controller.moveit2_robot0.move_to_pose(position=home_pos, quat_xyzw=[
                                                1.0, 0.0, 0.0, 0.0], cartesian=True, frame_id='world')
        self.robot_controller.moveit2_robot0.wait_until_executed()
        return 'wait'

class Wait(RosState):
    def __init__(self, robot_controller):
        State.__init__(self, outcomes=['approach','failed'])
        self.robot_controller=robot_controller

    def execute(self, userdata):
        self.robot_controller.last_pose = None
        while self.robot_controller.last_pose is None:
            time.sleep(0.1)

        return 'approach'
    
class ApproachObject(RosState):
    def __init__(self, robot_controller):
        State.__init__(self, outcomes=['approached', 'failed'])
        self.robot_controller=robot_controller

    def execute(self, userdata):
        target_pos = [self.robot_controller.last_pose.pose.position.x - .18, -0.4, 0.011]
        quat_xyzw = [1.0, 0.0, 0.0, 0.0]
        self.robot_controller.logger.info(f"Execution going to  {target_pos}, {self.robot_controller.last_pose.pose.position.y}")
        self.robot_controller.moveit2_robot0.move_to_pose(position=target_pos, quat_xyzw=quat_xyzw, cartesian=True)
        self.robot_controller.moveit2_robot0.wait_until_executed()
        return 'approached'
        #self.robot_controller.logger.info(f"Execution completed with status ={self.robot_controller.moveit2_robot0.move_action_status}")
        #if self.robot_controller.moveit2_robot0.move_action_status == GoalStatus.STATUS_SUCCEEDED:
        #     return 'approached'
        #else:
        #     return 'failed'
    

class GraspObject(RosState):
    def __init__(self, robot_controller):
        State.__init__(self, outcomes=['grasped','failed'])
        self.robot_controller=robot_controller

    def execute(self, userdata):
        self.robot_controller.logger.info(f"Before grasp")
        self.robot_controller.gripper_on()
        pose = self.robot_controller.last_pose
        while self.robot_controller.grasping == False and pose.name == self.robot_controller.last_pose.name:
             time.sleep(.1)

        
        '''
        while self.robot_controller.last_pose.pose.position.y < 0:
                self.robot_controller.logger.info(f"After grasp {self.robot_controller.last_pose.pose.position.y }")
                if self.robot_controller.grasping==True:
                  self.robot_controller.logger.info(f"Grasp successful")
                  return 'grasped'
                time.sleep(.1)
     
        # If unsuccessful:
        self.robot_controller.gripper_off()
        '''
        return 'grasped'

class PlaceObject(RosState):
    def __init__(self, robot_controller):
        State.__init__(self, outcomes=['placed','failed'])
        self.robot_controller=robot_controller

    def execute(self, userdata):
        # Code to place the object
        # This might use another MoveIt2 action, for example
        # If successful:
        pos1 = entity_location(self.robot_controller, "amr1", "arm1")
        self.turtlebot_pos = [pos1.position.x, pos1.position.y, 0.20]
        self.robot_controller.moveit2_robot0.move_to_pose(position=[self.robot_controller.last_pose.pose.position.x - .18, -0.3, 0.35], quat_xyzw=[
                                                 1.0, 0.0, 0.0, 0.0], cartesian=True)
        self.robot_controller.moveit2_robot0.wait_until_executed()
        if self.robot_controller.grasping==False:
             return 'failed'
        
        self.robot_controller.moveit2_robot0.move_to_pose(position=self.turtlebot_pos, quat_xyzw=[
                                                 1.0, 0.0, 0.0, 0.0], cartesian=True)
        self.robot_controller.moveit2_robot0.wait_until_executed()
        self.robot_controller.gripper_off()
        return 'placed'
        # If unsuccessful:
        return 'failed'

class TransferObject(RosState):
    def __init__(self, robot_controller):
        State.__init__(self, outcomes=['transferred','failed'])
        self.robot_controller=robot_controller

    def execute(self, userdata):
        # Code to place the object
        # This might use another MoveIt2 action, for example
        # If successful:
        self.robot_controller.amr_goto_pose(-3.0, 0.0, .004)
        return 'transferred'
     
    
def run_smach(client):
    # Create the top level SMACH state machine
    sm = StateMachine(outcomes=['succeeded', 'aborted'])
    with sm:
            StateMachine.add('SETUP', Setup(client), transitions={'home':'HOME', 'failed':'HOME'})
            StateMachine.add('HOME', Home(client), transitions={'wait':'WAIT', 'failed':'HOME'})
            StateMachine.add('WAIT', Wait(client), transitions={'approach':'APPROACH', 'failed':'HOME'})
            StateMachine.add('APPROACH', ApproachObject(client), transitions={'approached':'GRASP', 'failed':'HOME'})
            StateMachine.add('GRASP', GraspObject(client), transitions={'grasped':'PLACE', 'failed':'HOME'})
            StateMachine.add('PLACE', PlaceObject(client), transitions={'placed':'TRANSFER', 'failed':'HOME'})
            StateMachine.add('TRANSFER', TransferObject(client), transitions={'transferred':'HOME', 'failed':'HOME'})

    # Execute SMACH plan
    sm.execute()


def main(args=None):
   
    rclpy.init(args=args)
    args_without_ros = rclpy.utilities.remove_ros_args(args)
   
    robot_controller = RobotController(args_without_ros)
    robot_controller.get_logger().info('Robot Controller started')
    # Create and start the thread for running the state machine
    smach_thread = threading.Thread(target=run_smach, args=(robot_controller,))
    smach_thread.start()
  
    executor = MultiThreadedExecutor()
    executor.add_node(robot_controller)
    executor.spin()
    robot_controller.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

if __name__ == '__main__':
    main()

