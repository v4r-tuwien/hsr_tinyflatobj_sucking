#!/usr/bin/env python

import hsrb_interface
from hsrb_interface import Robot, geometry
import rospy
import sys
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi as pi
from std_srvs.srv import Empty


# Posture that 0.02[m] front and rotate -1.57 around z-axis of the bottle maker
bottle_to_hand = geometry.pose(z=-0.02, ek=-1.57)

# Posture to move the hand 0.1[m] up
hand_up = geometry.pose(x=0.1)

# Posture to move the hand 0.5[m] back
hand_back = geometry.pose(z=-0.5)

class ExecuteSuctionServer:
    def __init__(self):
        # Preparation for using the robot functions
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.gripper = self.robot.get('gripper')
        self.omni_base = self.robot.get('omni_base')

        # For coordinate transformations
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Initialize Moveit
        self.move_group = self.moveit_init()
        # For resetting the collision map
        self.clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)

        #self.server = actionlib.SimpleActionServer(
        #    'execute_grasp', ExecuteGraspAction, self.execute, False)
        #self.server.start()

        print('SuctionServer ready')


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

        #t = self.tf.getLatestCommonTime('/odom', '/base_link')
        #transform = self.tf.lookupTransform('/odom', '/base_link', t)
        #move_group.set_workspace(
        #    (-1.5 + transform[0][0], -1.5 + transform[0][1], -1, 1.5 + transform[0][0], 1.5 + transform[0][1], 3))

        move_group.allow_replanning(True)
        self.add_box('floor')
        #move_group.set_workspace((-20,-20,-1,20,20,4))
        # move_group.set_num_planning_attempts(5)
        # self.create_collision_environment()

        return move_group


    def execute(self, goal_frame):
        # Define the actual workspace as a 3x3x3 box with hsr in the middle
        hsr_pos = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time(), rospy.Duration(4.0))
        self.move_group.set_workspace(
           (-1.5 + hsr_pos.transform.translation.x, -1.5 + hsr_pos.transform.translation.y, -1, 1.5 + hsr_pos.transform.translation.x, 1.5 + hsr_pos.transform.translation.y, 3))

        # Reset the collision object map
        self.clear_octomap()
        rospy.sleep(1)

        print('Starting Moveit Commander')

        object_pos = self.tfBuffer.lookup_transform('odom', goal_frame, rospy.Time(), rospy.Duration(4.0))
        pose_goal = geometry_msgs.msg.Pose()
        [ang_x_axis, ang_y_axis, ang_z_axis] = euler_from_quaternion(
            [object_pos.transform.rotation.x, object_pos.transform.rotation.y, object_pos.transform.rotation.z,
             object_pos.transform.rotation.w])
        ang_x_axis = ang_x_axis - pi
        [pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z,
         pose_goal.orientation.w] = quaternion_from_euler(ang_x_axis, ang_y_axis, ang_z_axis)
        pose_goal.position.x = object_pos.transform.translation.x
        pose_goal.position.y = object_pos.transform.translation.y
        pose_goal.position.z = object_pos.transform.translation.z + 0.2

        self.move_group.set_pose_target(pose_goal)
        success = self.move_group.go(wait=True)
        print('Move successful: ' + str(success))
        self.move_group.stop()
        rospy.sleep(1)

        if success:
            self.whole_body.linear_weight = 100.0
            self.whole_body.end_effector_frame = u'hand_l_finger_vacuum_frame'
            self.whole_body.move_end_effector_pose(geometry.pose(ej=-pi), goal_frame)


    def add_box(self, name, position_x=0.0, position_y=0.0,
                position_z=0.0, size_x=0.1, size_y=0.1, size_z=0.1):
        """ Adds a box in the map frame to the MoveIt scene.

        Arguments:
            name {str}
            position_x {int} -- x coordinate in map frame (default: {0})
            position_y {int} -- y coordinate in map frame (default: {0})
            position_z {int} -- z coordinate in map frame (default: {0})
            size_x {float} -- size in x direction in meter (default: {0.1})
            size_y {float} -- size in y direction in meter (default: {0.1})
            size_z {float} -- size in z direction in meter (default: {0.1})
        """
        #rospy.sleep(0.2)
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "map"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = position_x
        box_pose.pose.position.y = position_y
        box_pose.pose.position.z = position_z
        box_name = name
        self.scene.add_box(box_name, box_pose, size=(size_x, size_y, size_z))

if __name__ == '__main__':
    rospy.init_node('execute_suction_server')
    server = ExecuteSuctionServer()

    rospy.sleep(1)

    try:
        #server.execute('my_frame')
        server.execute('found_object')
    except Exception as e:
        print(e)

    rospy.spin()






