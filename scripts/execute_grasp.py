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

# Move timeout[s]
_MOVE_TIMEOUT=60.0
# Grasp force[N]
_GRASP_FORCE=0.2




# Posture that 0.02[m] front and rotate -1.57 around z-axis of the bottle maker
bottle_to_hand = geometry.pose(z=-0.02, ek=-1.57)

# Posture to move the hand 0.1[m] up
hand_up = geometry.pose(x=0.1)

# Posture to move the hand 0.5[m] back
hand_back = geometry.pose(z=-0.5)


def add_box(name, position_x=0.0, position_y=0.0,
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
    scene.add_box(box_name, box_pose, size=(size_x, size_y, size_z))

if __name__ == '__main__':
    rospy.init_node('execute_grasp')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    #try:
        #object = tfBuffer.lookup_transform('map', 'found_object', rospy.Time(),rospy.Duration(6.0))
    #except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
     #   print(e)

    #print(object)

    # Preparation for using the robot functions
    robot = Robot()
    omni_base = robot.get('omni_base')
    whole_body = robot.try_get('whole_body')
    gripper = robot.get('gripper')
    #tts = robot.get('default_tts')

    print('Starting moveit')

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "whole_body"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )

    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the robot:
    #print("============ Printing robot state")
    #print(robot.get_current_state())
    #print("")

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0.65
    joint_goal[1] = -pi/2
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    #success = move_group.go(joint_goal, wait=True)

    object_pos = tfBuffer.lookup_transform('odom', 'my_frame', rospy.Time(), rospy.Duration(4.0))
    pose_goal = geometry_msgs.msg.Pose()
    [ang_x_axis, ang_y_axis, ang_z_axis] = euler_from_quaternion([object_pos.transform.rotation.x, object_pos.transform.rotation.y, object_pos.transform.rotation.z, object_pos.transform.rotation.w])
    ang_x_axis = ang_x_axis# - pi
    [pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w] = quaternion_from_euler(ang_x_axis, ang_y_axis, ang_z_axis)
    pose_goal.position.x = object_pos.transform.translation.x
    pose_goal.position.y = object_pos.transform.translation.y
    pose_goal.position.z = object_pos.transform.translation.z# + 0.2

    print(pose_goal)
    #print(object_pos)

    #move_group.set_workspace((10,10,10,1,1,1))
    add_box('floor', 0, 0, -0.1, 30, 30, 0.2)
    try:
        #rospy.wait_for_service('/clear_octomap')
        clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
        #rospy.sleep(1)
        clear_octomap()
    except Exception as e: print(e)
    #rospy.sleep(1)

    #move_group.set_pose_reference_frame('odom')
    move_group.set_pose_target(pose_goal)
    #move_group.allow_replanning(True)
    success = move_group.go(wait=True)
    print(success)
    move_group.stop()
    #move_group.clear_pose_targets()

    #rospy.spin()


