#!/usr/bin/env python3
# Copyright (C) 2016 Toyota Motor Corporation
from enum import Enum
from math import pi

import actionlib
import rospy
import smach
import smach_ros

#from actionlib_msgs.msg import GoalStatus

#from handover.msg import HandoverAction
from hsrb_interface import Robot
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import roslib
roslib.load_manifest('hsr_small_objects')
from hsr_small_objects.msg import FindObjectAction, PickObjectAction

# neutral joint positions
neutral_joint_positions = {'arm_flex_joint': 0.0,
                           'arm_lift_joint': 0.0,
                           'arm_roll_joint': -1.570,
                           'hand_motor_joint': 1.0,
                           'head_pan_joint': 0.0,
                           'head_tilt_joint': -0.75,
                           'wrist_flex_joint': -1.57,
                           'wrist_roll_joint': 0.0}

# Enum for states


class States(Enum):
    FIND_OBJECT = 1
    PICK_UP = 2
    LAY_DOWN = 3
    ACTIVATE_SUCTION = 4
    DEACTIVATE_SUCTION = 5
    RETURN_TO_NEUTRAL = 6
    SETTINGS = 7
    HELP = 8
    QUIT = 9



# Mapping of states to characters
states_keys = {States.FIND_OBJECT: 'f',
               States.PICK_UP: 'p',
               States.LAY_DOWN: 'l',
               States.ACTIVATE_SUCTION: 'a',
               States.DEACTIVATE_SUCTION: 'd',
               States.RETURN_TO_NEUTRAL: 'n',
               States.SETTINGS: 's',
               States.HELP: 'h',
               States.QUIT: 'q'}


class UserInput(smach.State):


    def __init__(self):
        smach.State.__init__(self, outcomes=['finding', 'picking', 'laying', 'start_suction', 'stop_suction',
                             'neutral', 'quit'],
                             input_keys=['retry', 'object_action', 'object_name', 'auto_continue'],
                             output_keys=['retry', 'object_action', 'object_name', 'auto_continue'])


    def execute(self, userdata):
        rospy.loginfo('Executing state UserInput')

        while not rospy.is_shutdown():
            self.print_info()

            while True:
                user_input = input('CMD> ')
                if len(user_input) == 1:
                    break
                print('Please enter only one character')
            char_in = user_input.lower()

            # Find Object
            if char_in == states_keys[States.FIND_OBJECT]:
                print('Finding object')
                return 'finding'

            # Picking up object
            elif char_in == states_keys[States.PICK_UP]:
                print('Picking up object')
                # TODO: check find
                return 'picking'

            # Laying down object
            elif char_in == states_keys[States.LAY_DOWN]:
                print('Laying down object')
                # TODO: check pickup
                return 'laying'

            # Start suction
            elif char_in == states_keys[States.ACTIVATE_SUCTION]:
                print('start_suction')
                return 'start_suction'

            # Stop suction
            elif char_in == states_keys[States.DEACTIVATE_SUCTION]:
                print('stop_suction')
                return 'stop_suction'

            # Return to neutral position
            elif char_in == states_keys[States.RETURN_TO_NEUTRAL]:
                print('Returning robot to neutral position')
                return 'neutral'

            # Settings
            elif char_in == states_keys[States.SETTINGS]:
                while not rospy.is_shutdown():
                    print('Settings:')
                    print('\t1 - Allow retries with base movement:\t\t' + str(userdata.retry))
                    print('\t2 - Action after pick-up:\t\t\t' + str(userdata.object_action))
                    print('\t3 - Change object, which should be found\t' + str(userdata.object_name))
                    print('\t4 - Continue automatic with the next action\t' + str(userdata.auto_continue))
                    print('')
                    print('\t5 - Back')
                    while True:
                        user_input = input('CMD> ')
                        if len(user_input) == 1:
                            break
                        print('Please enter only one character')
                    char_in = user_input.lower()
                    if char_in == '1':
                        if userdata.retry:
                            userdata.retry = False
                        else:
                            userdata.retry = True
                    elif char_in == '2':
                        if userdata.object_action == 'handover':
                            userdata.object_action = 'lay_down'
                        elif userdata.object_action == 'lay_down':
                            userdata.object_action = 'nothing'
                        else:
                            userdata.object_action = 'handover'
                    elif char_in == '3':
                        if userdata.object_name == 'card':
                            userdata.object_name = 'nix'
                        else:
                            userdata.object_name = 'card'
                    elif char_in == '4':
                        if userdata.auto_continue:
                            userdata.auto_continue = False
                        else:
                            userdata.auto_continue = True
                    elif char_in == '5':
                        break
                    else:
                        print('No valid option.')

            # Help
            elif char_in is None or char_in == states_keys[States.HELP]:
                print('\n\n\tFind_Object - HSR uses RGB and depth image to find and localize known flat objects.')

            # Quit
            elif char_in is None or char_in == states_keys[States.QUIT]:
                print('Tschau')
                return 'quit'

            # Unrecognized command
            else:
                print('Command not known, please give other command')

    def print_info(self):
        """ prints states_keys and their names
        e.g.:
        states_keys = {States.EXAMPLE: 'a',
                    States.ANOTHER_ONE: 'b'}
        output:
               a - EXAMPLE
               b - ANOTHER_ONE
                ...
        """
        for name, member in States.__members__.items():
            print(states_keys[member] + ' - ' + name)



class LayObject(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Info')
        print('Object abgelegt')
        return 'succeeded'

class StartSuction(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot = Robot()
        self.suction = self.robot.try_get('suction')

    def execute(self, userdata):
        print(50*'#' + '\n')
        if self.suction != None:
            rospy.loginfo('Starting suction')
            self.suction.command(True)
        else:
            rospy.loginfo('Could not start suction, is hsr connection established?')
        print(50*'#' + '\n')
        return 'succeeded'

class StopSuction(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot = Robot()
        self.suction = self.robot.try_get('suction')

    def execute(self, userdata):
        print(50*'#' + '\n')
        if self.suction != None:
            rospy.loginfo('Stopping suction')
            self.suction.command(False)
        else:
            rospy.loginfo('Could not stop suction, is hsr connection established?')
        print(50 * '#' + '\n')
        return 'succeeded'

class GoToNeutral(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')

    def execute(self, userdata):
        print(50 * '#' + '\n')
        if self.whole_body != None:
            rospy.loginfo('Returning to neutral position')
            self.whole_body.move_to_joint_positions(neutral_joint_positions)
            rospy.sleep(1.0)
            vel = self.whole_body.joint_velocities
            while all(abs(i) > 0.05 for i in vel.values()):
                vel = self.whole_body.joint_velocities
            rospy.sleep(2)
        else:
            rospy.loginfo('Could not return to neutral, is hsr connection established?')
        print(50 * '#' + '\n')
        return 'succeeded'

class Quit(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self):
        rospy.loginfo('Info')
        print('Abschalten')
        return 'succeeded'

class NextStep(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['finding', 'picking', 'laying', 'neutral', 'user_input'],
                             input_keys=['retry', 'object_action', 'auto_continue', 'result_info'])

    def execute(self, userdata):
        print(50*'#' + '\n')
        rospy.loginfo('Planning next step')
        out = 'user_input'

        if userdata.result_info == 'find_object_succeeded':
            if userdata.auto_continue == True:
                out = 'picking'
            else:
                out = 'user_input'

        if userdata.result_info == 'pick_object_succeeded':
            if userdata.auto_continue == True:
                if userdata.object_action == 'lay_down':
                    out = 'laying'
                elif userdata.object_action == 'handover':
                    out = 'handover'
                else:
                    out = 'user_input'
            else:
                out = 'user_input'

        print(50 * '#' + '\n')
        return out


def main():
    rospy.init_node('small_objects_statemachine')

    sm = smach.StateMachine(outcomes=['end'])

    # define some userdata
    sm.userdata.retry = True
    sm.userdata.object_action = 'handover'
    sm.userdata.object_name = 'card'
    sm.userdata.auto_continue = False
    sm.userdata.result_info = ''

    with sm:
        smach.StateMachine.add('USER_INPUT',
                               UserInput(),
                               transitions={'finding': 'Find_Object',
                                            'picking': 'Pick_Object',
                                            'laying': 'Lay_Object',
                                            'start_suction': 'Start_Suction',
                                            'stop_suction': 'Stop_Suction',
                                            'neutral': 'Go_To_Neutral',
                                            'quit': 'end'},
                               remapping={'retry': 'retry',
                                          'object_action': 'object_action',
                                          'object_name': 'object_name'})

        smach.StateMachine.add('Find_Object',
                               smach_ros.SimpleActionState('Find_Object', FindObjectAction,
                                                           goal_slots=['object_name'],
                                                           result_slots=['result_info'],),
                               transitions={'succeeded': 'Next_Step',
                                            'preempted': 'USER_INPUT',
                                            'aborted': 'USER_INPUT'},
                               remapping={'object_name': 'object_name',
                                          'result_info': 'result_info'})

        smach.StateMachine.add('Pick_Object',
                               smach_ros.SimpleActionState('Pick_Object', PickObjectAction,
                                                           goal_slots=['pick_object'],
                                                           result_slots=['result_info'],),
                               transitions={'succeeded': 'Next_Step',
                                            'preempted': 'USER_INPUT',
                                            'aborted': 'USER_INPUT'},
                               remapping={'pick_object': 'object_name',
                                          'result_info': 'result_info'})

        smach.StateMachine.add('Lay_Object',
                               LayObject(),
                               transitions={'succeeded': 'USER_INPUT'})

        smach.StateMachine.add('Start_Suction',
                               StartSuction(),
                               transitions={'succeeded': 'USER_INPUT'})

        smach.StateMachine.add('Stop_Suction',
                               StopSuction(),
                               transitions={'succeeded': 'USER_INPUT'})

        smach.StateMachine.add('Go_To_Neutral',
                               GoToNeutral(),
                               transitions={'succeeded': 'USER_INPUT'})

        smach.StateMachine.add('Next_Step',
                               NextStep(),
                               transitions={'finding': 'Find_Object',
                                            'picking': 'Pick_Object',
                                            'laying': 'Lay_Object',
                                            'neutral': 'Go_To_Neutral',
                                            'user_input': 'USER_INPUT'},
                               remapping={'retry': 'retry',
                                          'object_action': 'object_action',
                                          'auto_continue': 'auto_continue',
                                          'result_info': 'result_info'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('small_objects_statemachine_introspection_server', sm, '/SM_ROOT')
    sis.start()

    # Execute state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    # rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
