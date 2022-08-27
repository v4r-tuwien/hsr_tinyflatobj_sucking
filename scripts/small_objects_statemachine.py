#!/usr/bin/env python3
# Copyright (C) 2016 Toyota Motor Corporation
from enum import Enum
from math import pi

import actionlib
import rospy
import smach
import smach_ros

#from actionlib_msgs.msg import GoalStatus

from handover.msg import HandoverAction
from hsrb_interface import Robot
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import roslib
roslib.load_manifest('hsr_small_objects')
from hsr_small_objects.msg import FindObjectAction, PickObjectAction, FindObjectGoal, PickObjectGoal

# neutral joint positions
neutral_joint_positions = {'arm_flex_joint': 0.0,
                           'arm_lift_joint': 0.0,
                           'arm_roll_joint': -1.570,
                           'hand_motor_joint': 1.0,
                           'head_pan_joint': 0.0,
                           'head_tilt_joint': -0.75,
                           'wrist_flex_joint': -1.57,
                           'wrist_roll_joint': 0.0}


def create_waypoint(x, y, q_z, q_w):
    move_goal = MoveBaseGoal()
    move_goal.target_pose.header.frame_id = 'map'
    move_goal.target_pose.pose.position.x = x
    move_goal.target_pose.pose.position.y = y
    move_goal.target_pose.pose.orientation.z = q_z
    move_goal.target_pose.pose.orientation.w = q_w
    return move_goal


waypoint_1 = create_waypoint(0, -1, -0.707, 0.707)
waypoint_2 = create_waypoint(1, -1.3, -0.9489846, 0.3153224)
waypoints = [waypoint_1, waypoint_2]

lay_down_point = create_waypoint(0.6, -0.4, 0, 0)
handover_point = create_waypoint(0, 0, 0, 0)


class MoveToNextWaypoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'waypoint_not_reached', 'no_more_waypoints'],
                             input_keys=['waypoint_number'],
                             output_keys=['waypoint_number'])
        self.move_client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

    def execute(self, userdata):
        print(50 * '#' + '\n')
        if userdata.waypoint_number == len(waypoints):
            rospy.loginfo('No more waypoints')
            out = 'no_more_waypoints'
        else:
            self.move_client.wait_for_server(rospy.Duration(10.0))
            self.move_client.send_goal(waypoints[userdata.waypoint_number])
            result = self.move_client.wait_for_result(rospy.Duration(15.0))
            userdata.waypoint_number += 1
            if result:
                rospy.loginfo('Waypoint reached')
                out = 'succeeded'
            else:
                rospy.loginfo('Waypoint not reached')
                out = 'waypoint_not_reached'
        print('\n' + 50 * '#')
        return out


class FindObject(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'not_found', 'server_problem'],
                             input_keys=['object_name'])
        self.client = actionlib.SimpleActionClient('Find_Object_Action_Server', FindObjectAction)

    def execute(self, userdata):
        print(50 * '#' + '\n')
        goal = FindObjectGoal(object_name=userdata.object_name)
        self.client.wait_for_server(rospy.Duration(8.0))
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(8.0))
        find_result = self.client.get_result()
        if find_result.result_info == 'not found':
            rospy.loginfo('Object ' + str(userdata.object_name) + ' was not found')
            out = 'not_found'
        elif find_result.result_info == 'succeeded':
            rospy.loginfo('Found object ' + str(userdata.object_name))
            out = 'succeeded'
        else:
            rospy.loginfo('find_object_action_server problem')
            out = 'server_problem'
        print('\n' + 50 * '#')
        return out


class PickObject(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'movement_aborted', 'no_path_found', 'server_problem'],
                             input_keys=['object_name'])
        self.client = actionlib.SimpleActionClient('Pick_Object_Action_Server', PickObjectAction)

    def execute(self, userdata):
        print(50 * '#' + '\n')
        goal = PickObjectGoal(object_name=userdata.object_name)
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(20.0))
        pick_result = self.client.get_result()
        if pick_result.result_info == 'succeeded':
            rospy.loginfo('Picked object ' + str(userdata.object_name))
            out = 'succeeded'
        elif pick_result.result_info == 'movement_aborted':
            rospy.loginfo('Movement failed during execution')
            out = 'movement_aborted'
        elif pick_result.result_info == 'no_path_found':
            rospy.loginfo('No path was found from actual waypoint')
            out = 'no_path_found'
            # TODO: no path checken
        else:
            rospy.loginfo('pick_object_action_server problem')
            out = 'server_problem'
        print('\n' + 50 * '#')
        return out


class CheckPick(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_object_picked', 'robot_problem'])
        self.robot = Robot()
        self.suction = self.robot.try_get('suction')

    def execute(self, userdata):
        print(50 * '#' + '\n')
        if self.suction != None:
            check = self.suction.pressure_sensor
            if check:
                rospy.loginfo('Pick was successful')
                out = 'succeeded'
            else:
                rospy.loginfo('Object was not picked')
                out = 'no_object_picked'
        else:
            rospy.loginfo('Could find suction cup, is hsr connection established?')
            out = 'robot_problem'
        print('\n' + 50 * '#')
        return out


class MoveToDropPoint(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['hand_over_object', 'lay_down_object', 'drop_point_not_reached'],
                             input_keys=['object_action'])
        self.move_client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

    def execute(self, userdata):
        print(50 * '#' + '\n')
        if userdata.object_action == 'handover':
            out = 'hand_over_object'
            goal = handover_point
        else:
            out = 'lay_down_object'
            goal = lay_down_point
        self.move_client.wait_for_server(rospy.Duration(10.0))
        self.move_client.send_goal(goal)
        result = self.move_client.wait_for_result(rospy.Duration(15.0))
        if result:
            rospy.loginfo('Drop point reached')
        else:
            rospy.loginfo('Drop point not reached')
            out = 'drop_point_not_reached'
        print('\n' + 50 * '#')
        return out


class LayDown(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        print(50 * '#' + '\n')
        rospy.loginfo('Info')
        print('\n' + 50 * '#')
        return 'succeeded'


class Handover(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        # TODO: Handover machen

    def execute(self, userdata):
        print(50 * '#' + '\n')
        rospy.loginfo('Info')
        print('\n' + 50 * '#')
        return 'succeeded'


class GoToNeutral(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'robot_problem'])
        self.robot = Robot()
        try:
            self.whole_body = self.robot.try_get('whole_body')
        except Exception as e:
            rospy.loginfo('Problem finding: ' + str(e))
            self.whole_body = None

    def execute(self, userdata):
        print(50 * '#' + '\n')
        if self.whole_body != None:
            rospy.loginfo('Returning to neutral position')
            self.whole_body.move_to_joint_positions(neutral_joint_positions)
            rospy.sleep(1.0)
            vel = self.whole_body.joint_velocities
            while all(abs(i) > 0.05 for i in vel.values()):
                vel = self.whole_body.joint_velocities
            rospy.sleep(1)
            out = 'succeeded'

        else:
            rospy.loginfo('Could not return to neutral, is hsr connection established?')
            out = 'robot_problem'
        print('\n' + 50 * '#')
        return out


class MoveToLastWaypoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'waypoint_not_reached'],
                             input_keys=['waypoint_number'])
        self.move_client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

    def execute(self, userdata):
        print(50 * '#' + '\n')
        self.move_client.wait_for_server(rospy.Duration(10.0))
        self.move_client.send_goal(waypoints[userdata.waypoint_number-1])
        result = self.move_client.wait_for_result(rospy.Duration(15.0))
        if result:
            rospy.loginfo('Waypoint reached')
            out = 'succeeded'
        else:
            rospy.loginfo('Waypoint not reached')
            out = 'waypoint_not_reached'
        print('\n' + 50 * '#')
        return out


def create_small_objects_sm():

    sm = smach.StateMachine(outcomes=['Exit'])

    # define some userdata
    sm.userdata.object_action = 'handover'
    sm.userdata.object_name = 'card'
    sm.userdata.waypoint_number = 0

    with sm:

        smach.StateMachine.add('Move_To_Next_Waypoint',
                               MoveToNextWaypoint(),
                               transitions={'succeeded': 'Find_Object',
                                            'no_more_waypoints': 'Exit',
                                            'waypoint_not_reached': 'Move_To_Next_Waypoint'},
                               remapping={'waypoint_number': 'waypoint_number'})

        smach.StateMachine.add('Find_Object',
                               FindObject(),
                               transitions={'succeeded': 'Pick_Object',
                                            'server_problem': 'Exit',
                                            'not_found': 'Move_To_Next_Waypoint'},
                               remapping={'object_name': 'object_name'})

        smach.StateMachine.add('Pick_Object',
                               PickObject(),
                               transitions={'succeeded': 'Check_Pick',
                                            'server_problem': 'Exit',
                                            'movement_aborted': 'Go_To_Neutral',
                                            'no_path_found': 'Move_To_Next_Waypoint'},
                               remapping={'object_name': 'object_name'})

        smach.StateMachine.add('Check_Pick',
                               CheckPick(),
                               transitions={'succeeded': 'Move_To_Drop_Point',
                                            'no_object_picked': 'Move_To_Last_Waypoint',
                                            'robot_problem': 'Exit'})

        smach.StateMachine.add('Move_To_Drop_Point',
                               LayDown(),
                               transitions={'lay_down_object': 'Lay_Down',
                                            'hand_over_object': 'Handover',
                                            'drop_point_not_reached': 'Exit'},
                               remapping={'object_action': 'object_action'})

        smach.StateMachine.add('Lay_Down',
                               LayDown(),
                               transitions={'succeeded': 'Exit'})

        smach.StateMachine.add('Handover',
                               Handover(),
                               transitions={'succeeded': 'Exit'})

        smach.StateMachine.add('Go_To_Neutral',
                               GoToNeutral(),
                               transitions={'succeeded': 'Pick_Object',
                                            'robot_problem': 'Exit'})

        smach.StateMachine.add('Move_To_Last_Waypoint',
                               MoveToLastWaypoint(),
                               transitions={'succeeded': 'Pick_Object',
                                            'waypoint_not_reached': 'Move_To_Next_Waypoint'})
    return sm
