#!/usr/bin/env python


from hsrb_interface import Robot, geometry
import rospy
import sys
import tf2_ros
#import tf2_msgs.msg
import geometry_msgs.msg
import tf2_geometry_msgs.tf2_geometry_msgs

import moveit_commander
import moveit_msgs.msg


from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix
from math import pi as pi
from math import cos, sin
from std_srvs.srv import Empty

from hsr_small_objects.msg import ArmMovementAction, ArmMovementActionResult
import actionlib
import roslib
roslib.load_manifest('hsr_small_objects')



class ArmMovementActionServer:
    def __init__(self):
        # Prepare action server
        self.server = actionlib.SimpleActionServer('Arm_Movement_Action_Server', ArmMovementAction, self.execute, False)
        self.server.start()

        # Preparation for using the robot functions
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.suction = self.robot.get('suction')
        self.omni_base = self.robot.get('omni_base')

        # For coordinate transformations
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Initialize Moveit
        self.move_group = self.moveit_init()
        # For resetting the collision map
        self.clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)


    def moveit_init(self):
        """ Initializes MoveIt, sets workspace and creates collision environment

        Returns:
            MoveGroupCommander -- MoveIt interface
        """
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot_cmd = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "whole_body"
        move_group = moveit_commander.MoveGroupCommander(self.group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        # Print to get information about the moveit group
        self.planning_frame = move_group.get_planning_frame()
        self.eef_link = move_group.get_end_effector_link()
        self.group_names = self.robot_cmd.get_group_names()

        move_group.allow_replanning(True)
        self.add_floor()
        # move_group.set_num_planning_attempts(5)

        return move_group


    def execute(self, goal):
        if 'lay_down' == goal.command:
            rospy.loginfo('Execute lay_down')
            self.lay_down()
        else:
            rospy.loginfo('Execute pick up ' + goal.command)
            self.pick_object(goal.command)



    def pick_object(self, object_name):
        # define result of action_sever
        result = ArmMovementActionResult().result
        # check if found_object frame is published
        try:
            test = self.tfBuffer.lookup_transform('odom', 'found_object_' + object_name, rospy.Time(), rospy.Duration(4.0))
        except Exception as e:
            if 'source_frame does not exist' in str(e):
                result.result_info = 'goal_frame_missing'
                rospy.loginfo('goal_frame_missing')
            self.server.set_succeeded(result)
            return

        # Define the actual workspace as a 3x3x3 box with hsr in the middle
        hsr_pos = self.tfBuffer.lookup_transform('odom', 'base_link', rospy.Time(), rospy.Duration(4.0))
        self.move_group.set_workspace(
           (-1.5 + hsr_pos.transform.translation.x, -1.5 + hsr_pos.transform.translation.y, -1, 1.5 +
            hsr_pos.transform.translation.x, 1.5 + hsr_pos.transform.translation.y, 3))

        # Reset the collision object map
        self.clear_octomap()
        rospy.sleep(0.5)


        # Distance between hsr hand and suction cup
        hand2cup = self.tfBuffer.lookup_transform('hand_palm_link', 'hand_l_finger_vacuum_frame', rospy.Time(),
                                                  rospy.Duration(4.0))
        [hand2cup_ang_x_axis, hand2cup_ang_y_axis, hand2cup_ang_z_axis] = euler_from_quaternion(
            [hand2cup.transform.rotation.x, hand2cup.transform.rotation.y, hand2cup.transform.rotation.z,
             hand2cup.transform.rotation.w])

        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = 'found_object_' + object_name

        # hand2cup distance gets added to object position / therefore rotation around x-axis
        # z_offset from cup to object should be 0.05
        rot_angle = -pi - hand2cup_ang_x_axis
        pose_goal.pose.position.x = -hand2cup.transform.translation.x
        pose_goal.pose.position.y = -(cos(rot_angle) * hand2cup.transform.translation.y - sin(rot_angle) *
                                      hand2cup.transform.translation.z)
        pose_goal.pose.position.z = 0.05 - (sin(rot_angle) * hand2cup.transform.translation.y + cos(rot_angle) *
                                            hand2cup.transform.translation.z)
        pose_goal.pose.orientation.x = 0
        pose_goal.pose.orientation.y = 0
        pose_goal.pose.orientation.z = 0
        pose_goal.pose.orientation.w = 1

        [ang_x_axis, ang_y_axis, ang_z_axis] = euler_from_quaternion([pose_goal.pose.orientation.x,
            pose_goal.pose.orientation.y, pose_goal.pose.orientation.z, pose_goal.pose.orientation.w])
        ang_x_axis = ang_x_axis + rot_angle
        [pose_goal.pose.orientation.x, pose_goal.pose.orientation.y, pose_goal.pose.orientation.z,
            pose_goal.pose.orientation.w] = quaternion_from_euler(ang_x_axis, ang_y_axis, ang_z_axis)

        # moveit needs msg.Pose type
        pose_goal_odom = self.tfBuffer.transform(pose_goal, 'odom', rospy.Duration(4.0))
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = pose_goal_odom.pose.position.x
        pose_goal.position.y = pose_goal_odom.pose.position.y
        pose_goal.position.z = pose_goal_odom.pose.position.z
        pose_goal.orientation.x = pose_goal_odom.pose.orientation.x
        pose_goal.orientation.y = pose_goal_odom.pose.orientation.y
        pose_goal.orientation.z = pose_goal_odom.pose.orientation.z
        pose_goal.orientation.w = pose_goal_odom.pose.orientation.w

        self.publish_pose(pose_goal)
        self.move_group.set_pose_target(pose_goal)
        moveit_success = self.move_group.go(wait=True)
        self.move_group.stop()
        rospy.sleep(0.5)

        if moveit_success:
            # set weights to make sure robot does not use base movement
            self.whole_body.linear_weight = 100.0
            self.whole_body.angular_weight = 100.0
            self.whole_body.end_effector_frame = u'hand_l_finger_vacuum_frame'
            self.suction.command(True)
            self.whole_body.move_end_effector_by_line((0, 0, 1), 0.05)
            # TODO: Check force sensor
            rospy.sleep(0.25)
            self.whole_body.move_end_effector_by_line((0, 0, 1), -0.1)

            result.result_info = 'succeeded'
            rospy.loginfo('pick succeeded')
            self.server.set_succeeded(result)

        else:
            result.result_info = 'movement_failed'
            rospy.loginfo('movement_failed')
            self.server.set_succeeded(result)


    def lay_down(self):
        # define result of action_sever
        result = ArmMovementActionResult().result
        # check if marker frame for storage_box is published
        try:
            marker_pos = self.tfBuffer.lookup_transform('odom', 'ar_marker/508', rospy.Time(),
                                                  rospy.Duration(4.0))
        except Exception as e:
            if 'source_frame does not exist' in str(e):
                rospy.loginfo('goal_frame_missing')
                result.result_info = 'goal_frame_missing'
            self.server.set_succeeded(result)
            return

        # Define the actual workspace as a 3x3x3 box with hsr in the middle
        hsr_pos = self.tfBuffer.lookup_transform('odom', 'base_link', rospy.Time(), rospy.Duration(4.0))
        self.move_group.set_workspace(
            (-1.5 + hsr_pos.transform.translation.x, -1.5 + hsr_pos.transform.translation.y, -1, 1.5 +
             hsr_pos.transform.translation.x, 1.5 + hsr_pos.transform.translation.y, 3))

        # Reset the collision object map
        self.clear_octomap()
        rospy.sleep(0.5)

        box = geometry_msgs.msg.TransformStamped()
        box.header.frame_id = 'ar_marker/508'
        box.header.stamp = rospy.Time.now()
        box.child_frame_id = 'storage_box'
        box.transform.translation.z = 0.1
        [box.transform.rotation.x, box.transform.rotation.y, box.transform.rotation.z,
         box.transform.rotation.w] = quaternion_from_euler(pi/2, pi/2, 0)
        self.broadcaster.sendTransform(box)
        rospy.sleep(0.5)

        # same rotations as in the pick_object function
        # Distance between hsr hand and suction cup
        hand2cup = self.tfBuffer.lookup_transform('hand_palm_link', 'hand_l_finger_vacuum_frame', rospy.Time(),
                                                  rospy.Duration(4.0))
        [hand2cup_ang_x_axis, hand2cup_ang_y_axis, hand2cup_ang_z_axis] = euler_from_quaternion(
            [hand2cup.transform.rotation.x, hand2cup.transform.rotation.y, hand2cup.transform.rotation.z,
             hand2cup.transform.rotation.w])

        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = 'storage_box'

        # hand2cup distance gets added to object position / therefore rotation around x-axis
        # z_offset from cup to box should be 0.1
        rot_angle = -pi - hand2cup_ang_x_axis
        pose_goal.pose.position.x = -hand2cup.transform.translation.x
        pose_goal.pose.position.y = -(cos(rot_angle) * hand2cup.transform.translation.y - sin(rot_angle) *
                                      hand2cup.transform.translation.z)
        pose_goal.pose.position.z = 0.1 - (sin(rot_angle) * hand2cup.transform.translation.y + cos(rot_angle) *
                                            hand2cup.transform.translation.z)
        pose_goal.pose.orientation.x = 0
        pose_goal.pose.orientation.y = 0
        pose_goal.pose.orientation.z = 0
        pose_goal.pose.orientation.w = 1

        [ang_x_axis, ang_y_axis, ang_z_axis] = euler_from_quaternion([pose_goal.pose.orientation.x,
                                                                      pose_goal.pose.orientation.y,
                                                                      pose_goal.pose.orientation.z,
                                                                      pose_goal.pose.orientation.w])
        ang_x_axis = ang_x_axis + rot_angle
        [pose_goal.pose.orientation.x, pose_goal.pose.orientation.y, pose_goal.pose.orientation.z,
         pose_goal.pose.orientation.w] = quaternion_from_euler(ang_x_axis, ang_y_axis, ang_z_axis)

        # moveit needs msg.Pose type
        pose_goal_odom = self.tfBuffer.transform(pose_goal, 'odom', rospy.Duration(4.0))
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = pose_goal_odom.pose.position.x
        pose_goal.position.y = pose_goal_odom.pose.position.y
        pose_goal.position.z = pose_goal_odom.pose.position.z
        pose_goal.orientation.x = pose_goal_odom.pose.orientation.x
        pose_goal.orientation.y = pose_goal_odom.pose.orientation.y
        pose_goal.orientation.z = pose_goal_odom.pose.orientation.z
        pose_goal.orientation.w = pose_goal_odom.pose.orientation.w

        self.publish_pose(pose_goal)
        self.move_group.set_pose_target(pose_goal)
        moveit_success = self.move_group.go(wait=True)
        self.move_group.stop()
        rospy.sleep(0.5)

        if moveit_success:
            # Drop object
            self.suction.command(False)
            arm_angle = self.whole_body.joint_positions['arm_roll_joint']
            for i in range(0,5):
                self.whole_body.move_to_joint_positions({'arm_roll_joint': arm_angle + 0.1})
                self.whole_body.move_to_joint_positions({'arm_roll_joint': arm_angle - 0.1})
            rospy.sleep(1)

            if not self.suction.pressure_sensor:
                rospy.loginfo('lay_down succeeded')
                result.result_info = 'succeeded'
                self.server.set_succeeded(result)
            else:
                rospy.loginfo('lay_down failed, object still on cup')
                result.result_info = 'object_still_on_cup'
                self.server.set_succeeded(result)

        else:
            result.result_info = 'movement_failed'
            rospy.loginfo('movement_failed')
            self.server.set_succeeded(result)

# TODO: add table position function
    def add_floor(self, name='floor', position_x=0.0, position_y=0.0,
                position_z=-0.07, size_x=20, size_y=20, size_z=0.1):
        # Creates a flat box under the robot to represent the floor
        # Octomap voxels get neglected around normal collision objects, without the floor moveit detects a collision

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "map"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = position_x
        box_pose.pose.position.y = position_y
        box_pose.pose.position.z = position_z
        box_name = name
        self.scene.add_box(box_name, box_pose, size=(size_x, size_y, size_z))
        rospy.sleep(0.2)
        # TODO: check this add_floor function

    def publish_pose(self, pose_frame):

        transform_frame = geometry_msgs.msg.TransformStamped()
        transform_frame.header.frame_id = 'odom'
        transform_frame.header.stamp = rospy.Time.now()

        transform_frame.child_frame_id = 'moveit_goal_frame'
        transform_frame.transform.translation.x = pose_frame.position.x
        transform_frame.transform.translation.y = pose_frame.position.y
        transform_frame.transform.translation.z = pose_frame.position.z

        transform_frame.transform.rotation.x = pose_frame.orientation.x
        transform_frame.transform.rotation.y = pose_frame.orientation.y
        transform_frame.transform.rotation.z = pose_frame.orientation.z
        transform_frame.transform.rotation.w = pose_frame.orientation.w

        self.broadcaster.sendTransform(transform_frame)


if __name__ == '__main__':

    rospy.init_node('arm_movement_action_server')
    server = ArmMovementActionServer()
    print('arm_movement_action_server started')
    rospy.spin()






