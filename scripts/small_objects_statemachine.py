#!/usr/bin/env python

from enum import Enum
from math import pi

import actionlib
import rospy
import smach
import smach_ros


from handover.msg import HandoverAction
from hsrb_interface import Robot, geometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import roslib
roslib.load_manifest('hsr_small_objects')
from hsr_small_objects.msg import FindObjectAction, FindObjectGoal, ArmMovementAction, ArmMovementGoal
from handover.msg import HandoverAction

# neutral joint positions
neutral_joint_positions = {'arm_flex_joint': 0.0,
                           'arm_lift_joint': 0.0,
                           'arm_roll_joint': -1.570,
                           'hand_motor_joint': 1.0,
                           'head_pan_joint': 0.0,
                           'head_tilt_joint': -0.75,
                           'wrist_flex_joint': -1.57,
                           'wrist_roll_joint': 0.0}


class MoveToNextWaypoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'waypoint_not_reached', 'no_more_waypoints'],
                             input_keys=['waypoint_number', 'map'],
                             output_keys=['waypoint_number'])
        self.move_client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        self.robot = Robot()
        try:
            self.whole_body = self.robot.try_get('whole_body')
        except Exception as e:
            rospy.loginfo('Problem finding: ' + str(e))
            self.whole_body = None

        # self.hsr_pos_x = -1000
        # self.hsr_pos_y = -1000

    def execute(self, userdata):
        print(50 * '#' + '\n')
        if userdata.waypoint_number == len(userdata.map.waypoints):
            rospy.loginfo('No more waypoints')
            out = 'no_more_waypoints'
        else:
            server_check = self.move_client.wait_for_server(rospy.Duration(10.0))
            if not server_check:
                rospy.loginfo('move_base server not active')
                return 'no_more_waypoints'
            self.move_client.send_goal(userdata.map.waypoints[userdata.waypoint_number],
                                       feedback_cb=self.callback_feedback)
            result = self.move_client.wait_for_result(rospy.Duration(40.0))
            userdata.waypoint_number += 1
            if result:
                rospy.loginfo('Waypoint reached')
                out = 'succeeded'
                if self.whole_body is not None:
                    self.whole_body.gaze_point(point=geometry.Vector3(x=1.0, y=0.0, z=0.5), ref_frame_id='base_link')
                else:
                    rospy.loginfo('Could not gaze at point, is hsr connection established?')
            else:
                rospy.loginfo('Waypoint not reached')
                out = 'waypoint_not_reached'
        print('\n' + 50 * '#')
        return out

    def callback_feedback(self, feedback):
        # TODO: cancel move_base goal
        pass
        #if self.hsr_pos_x == round(feedback.base_position.pose.position.x, 3):
        #    if self.hsr_pos_y == round(feedback.base_position.pose.position.y, 3):
        #        rospy.loginfo('Waypoint not reached')
        #self.hsr_pos_x = round(feedback.base_position.pose.position.x, 2)
        #self.hsr_pos_y = round(feedback.base_position.pose.position.y, 2)


class FindObject(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'not_found', 'server_problem'],
                             input_keys=['object_name'])
        self.client = actionlib.SimpleActionClient('Find_Object_Action_Server', FindObjectAction)

    def execute(self, userdata):
        print(50 * '#' + '\n')
        goal = FindObjectGoal(object_name=userdata.object_name)
        self.client.wait_for_server(rospy.Duration(15.0))
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(25.0))
        find_result = self.client.get_result()
        if find_result is None:
            return 'server_problem'
        if find_result.result_info == 'not_found':
            # TODO: anderer gaze point
            rospy.loginfo('Object ' + str(userdata.object_name) + ' was not found')
            out = 'not_found'
        elif find_result.result_info == 'succeeded':
            rospy.loginfo('Found object ' + str(userdata.object_name))
            out = 'succeeded'
        else:
            rospy.loginfo('Find_Object_Action_Server problem')
            out = 'server_problem'
        print('\n' + 50 * '#')
        return out


class PickObject(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'movement_aborted', 'no_path_found', 'server_problem'],
                             input_keys=['object_name'])
        self.client = actionlib.SimpleActionClient('Arm_Movement_Action_Server', ArmMovementAction)

    def execute(self, userdata):
        print(50 * '#' + '\n')
        goal = ArmMovementGoal(userdata.object_name)
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(25.0))
        pick_result = self.client.get_result()
        if pick_result is None:
            return 'server_problem'
        if pick_result.result_info == 'succeeded':
            rospy.loginfo('Picked object ' + str(userdata.object_name))
            out = 'succeeded'
        elif pick_result.result_info == 'movement_aborted':
            rospy.loginfo('Movement failed during execution')
            out = 'movement_aborted'
        elif pick_result.result_info == 'no_path_found':
            rospy.loginfo('No path was found from actual waypoint')
            out = 'no_path_found'
        else:
            rospy.loginfo('Arm_Movement_Action_Server problem')
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
                             input_keys=['object_action', 'map'])
        self.move_client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        self.robot = Robot()
        try:
            self.whole_body = self.robot.try_get('whole_body')
            self.omni_base = self.robot.try_get('omni_base')
        except Exception as e:
            rospy.loginfo('Problem finding: ' + str(e))
            self.whole_body = None
            self.omni_base = None

    def execute(self, userdata):
        print(50 * '#' + '\n')
        # go to neutral position
        if self.whole_body is not None and self.omni_base is not None:
            rospy.loginfo('Returning to neutral position')
            self.omni_base.go_rel(-0.15, 0.0, 0.0, 20.0)
            rospy.sleep(0.5)
            self.whole_body.move_to_joint_positions(neutral_joint_positions)
            rospy.sleep(1.0)
            vel = self.whole_body.joint_velocities
            while all(abs(i) > 0.05 for i in vel.values()):
                vel = self.whole_body.joint_velocities
            rospy.sleep(1)
        else:
            rospy.loginfo('Could not return to neutral, is hsr connection established?')
            return 'drop_point_not_reached'

        # go to drop point
        rospy.loginfo('Going to drop point')
        if userdata.object_action == 'handover':
            out = 'hand_over_object'
            goal = userdata.map.handover_point
        else:
            out = 'lay_down_object'
            goal = userdata.map.lay_down_point

        self.move_client.wait_for_server(rospy.Duration(10.0))
        self.move_client.send_goal(goal)
        result = self.move_client.wait_for_result(rospy.Duration(25.0))
        if result:
            rospy.loginfo('Drop point reached')
        else:
            rospy.loginfo('Drop point not reached')
            out = 'drop_point_not_reached'
        print('\n' + 50 * '#')
        return out


class LayDown(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'server_problem'])
        self.client = actionlib.SimpleActionClient('Arm_Movement_Action_Server', ArmMovementAction)

    def execute(self, userdata):
        print(50 * '#' + '\n')
        goal = ArmMovementGoal('lay_down')
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(25.0))
        lay_down_result = self.client.get_result()
        if lay_down_result.result_info == 'succeeded':
            rospy.loginfo('Picked object ' + str(userdata.object_name))
            out = 'succeeded'
        else:
            rospy.loginfo('Arm_Movement_Action_Server problem')
            out = 'server_problem'
        print('\n' + 50 * '#')
        return out


class Handover(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        # TODO: Handover wegen suction cup Pumpe checken

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
        if self.whole_body is not None:
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
        smach.State.__init__(self, outcomes=['succeeded', 'waypoint_not_reached', 'robot_problem'],
                             input_keys=['waypoint_number', 'map'])
        self.move_client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        self.robot = Robot()
        try:
            self.whole_body = self.robot.try_get('whole_body')
            self.omni_base = self.robot.try_get('omni_base')
        except Exception as e:
            rospy.loginfo('Problem finding: ' + str(e))
            self.whole_body = None
            self.omni_base = None

    def execute(self, userdata):
        print(50 * '#' + '\n')

        # go to neutral position
        if self.whole_body is not None and self.omni_base is not None:
            rospy.loginfo('Returning to neutral position')
            self.omni_base.go_rel(-0.15, 0.0, 0.0, 20.0)
            rospy.sleep(0.5)
            self.whole_body.move_to_joint_positions(neutral_joint_positions)
            rospy.sleep(1.0)
            vel = self.whole_body.joint_velocities
            while all(abs(i) > 0.05 for i in vel.values()):
                vel = self.whole_body.joint_velocities
            rospy.sleep(1)
        else:
            rospy.loginfo('Could not return to neutral, is hsr connection established?')
            return 'robot_problem'

        # move to last waypoint
        self.move_client.wait_for_server(rospy.Duration(10.0))
        self.move_client.send_goal(userdata.map.waypoints[userdata.waypoint_number-1])
        result = self.move_client.wait_for_result(rospy.Duration(25.0))
        if result:
            rospy.loginfo('Waypoint reached')
            out = 'succeeded'
        else:
            rospy.loginfo('Waypoint not reached')
            out = 'waypoint_not_reached'
        print('\n' + 50 * '#')
        return out


def create_small_objects_sm():

    sm = smach.StateMachine(outcomes=['Exit', 'Exit_successful'], input_keys=['object_action', 'object_name', 'map'])

    # define some userdata
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
                               MoveToDropPoint(),
                               transitions={'lay_down_object': 'Lay_Down',
                                            'hand_over_object': 'Handover',
                                            'drop_point_not_reached': 'Exit'},
                               remapping={'object_action': 'object_action'})

        smach.StateMachine.add('Lay_Down',
                               LayDown(),
                               transitions={'succeeded': 'Exit_successful',
                                            'server_problem': 'Exit'})

        smach.StateMachine.add('Handover', smach_ros.SimpleActionState('/handover', HandoverAction),
                               transitions={'succeeded': 'Exit_successful',
                                            'preempted': 'Exit',
                                            'aborted': 'Exit'})

        smach.StateMachine.add('Go_To_Neutral',
                               GoToNeutral(),
                               transitions={'succeeded': 'Pick_Object',
                                            'robot_problem': 'Exit'})

        smach.StateMachine.add('Move_To_Last_Waypoint',
                               MoveToLastWaypoint(),
                               transitions={'succeeded': 'Pick_Object',
                                            'waypoint_not_reached': 'Move_To_Next_Waypoint',
                                            'robot_problem': 'Exit'})
    return sm
