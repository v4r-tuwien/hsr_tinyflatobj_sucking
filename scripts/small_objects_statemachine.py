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
from hsr_small_objects.msg import FindObjectAction

# neutral joint positions
neutral_joint_positions = {'arm_flex_joint': 0.0,
                           'arm_lift_joint': 0.0,
                           'arm_roll_joint': 1.570,
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
                             input_keys=['config'],
                             output_keys=['config'])


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
                    print('\t1 - Allow retries with base movement:\t' + str(userdata.config.retry))
                    print('\t2 - Handover after pick-up:\t\t' + str(userdata.config.handover))
                    print('')
                    print('\t3 - Back')
                    while True:
                        user_input = input('CMD> ')
                        if len(user_input) == 1:
                            break
                        print('Please enter only one character')
                    char_in = user_input.lower()
                    if char_in == '1':
                        userdata.config.change_retry()
                    elif char_in == '2':
                        userdata.config.change_handover()
                    elif char_in == '3':
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


class FindObject(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Info')
        print('Object gefunden')
        return 'succeeded'

class PickObject(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Info')
        print('Object aufgehoben')
        return 'succeeded'

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

    def execute(self):
        rospy.loginfo('Info')
        print('Starte Pumpe')
        return 'succeeded'

class StopSuction(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self):
        rospy.loginfo('Info')
        print('Stoppe Pumpe')
        return 'succeeded'

class GoToNeutral(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Info')
        print('Neutralposition')
        return 'succeeded'

class Quit(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self):
        rospy.loginfo('Info')
        print('Abschalten')
        return 'succeeded'

class CheckError(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Info')
        print('Mhm was lief denn falsch?')
        return 'succeeded'

class config():
    def __init__(self):
        self.retry = True
        self.handover = False

    def change_retry(self):
        if self.retry == True:
            self.retry = False
        else:
            self.retry = True

    def change_handover(self):
        if self.handover == True:
            self.handover = False
        else:
            self.handover = True

def main():
    rospy.init_node('small_objects_statemachine')

    sm = smach.StateMachine(outcomes=['end'])
    sm.userdata.config = config()
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
                               remapping={'config': 'config'})

        smach.StateMachine.add('Find_Object',
                               FindObject(),
                               transitions={'succeeded': 'USER_INPUT'})

        smach.StateMachine.add('Pick_Object',
                               PickObject(),
                               transitions={'succeeded': 'USER_INPUT'})

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

        smach.StateMachine.add('Check_Error',
                               CheckError(),
                               transitions={'succeeded': 'USER_INPUT'})

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
