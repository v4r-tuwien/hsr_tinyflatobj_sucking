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
import small_objects_statemachine

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
    START_STATEMACHINE = 1
    FIND_OBJECT = 2
    PICK_UP = 3
    LAY_DOWN = 4
    ACTIVATE_SUCTION = 5
    DEACTIVATE_SUCTION = 6
    RETURN_TO_NEUTRAL = 7
    SETTINGS = 8
    HELP = 9
    QUIT = 10


# Mapping of states to characters
states_keys = {States.START_STATEMACHINE: 's',
               States.FIND_OBJECT: 'f',
               States.PICK_UP: 'p',
               States.LAY_DOWN: 'l',
               States.ACTIVATE_SUCTION: 'a',
               States.DEACTIVATE_SUCTION: 'd',
               States.RETURN_TO_NEUTRAL: 'n',
               States.SETTINGS: 'c',
               States.HELP: 'h',
               States.QUIT: 'q'}


class UserInput(smach.State):


    def __init__(self):
        smach.State.__init__(self, outcomes=['statemachine', 'finding', 'picking', 'laying', 'start_suction',
                                             'stop_suction', 'neutral', 'quit'],
                             input_keys=['object_action', 'object_name'],
                             output_keys=['object_action', 'object_name'])


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

            if char_in == states_keys[States.START_STATEMACHINE]:
                print('Starting statemachine')
                return 'statemachine'

            if char_in == states_keys[States.FIND_OBJECT]:
                print('Finding object')
                return 'finding'

            elif char_in == states_keys[States.PICK_UP]:
                print('Picking up object')
                return 'picking'

            elif char_in == states_keys[States.LAY_DOWN]:
                print('Laying down object')
                return 'laying'

            elif char_in == states_keys[States.ACTIVATE_SUCTION]:
                print('start_suction')
                return 'start_suction'

            elif char_in == states_keys[States.DEACTIVATE_SUCTION]:
                print('stop_suction')
                return 'stop_suction'

            elif char_in == states_keys[States.RETURN_TO_NEUTRAL]:
                print('Returning robot to neutral position')
                return 'neutral'

            elif char_in == states_keys[States.SETTINGS]:
                while not rospy.is_shutdown():
                    print('Settings:')
                    print('\t1 - Action after pick-up:\t\t\t' + str(userdata.object_action))
                    print('\t2 - Change object, which should be found\t' + str(userdata.object_name))
                    print('')
                    print('\t3 - Back')
                    while True:
                        user_input = input('CMD> ')
                        if len(user_input) == 1:
                            break
                        print('Please enter only one character')
                    char_in = user_input.lower()
                    if char_in == '1':
                        if userdata.object_action == 'handover':
                            userdata.object_action = 'lay_down'
                        elif userdata.object_action == 'lay_down':
                            userdata.object_action = 'nothing'
                        else:
                            userdata.object_action = 'handover'
                    elif char_in == '2':
                        if userdata.object_name == 'card':
                            userdata.object_name = 'nix'
                        else:
                            userdata.object_name = 'card'
                    elif char_in == '3':
                        break
                    else:
                        print('No valid option.')

            elif char_in is None or char_in == states_keys[States.HELP]:
                print('\n\n\tFind_Object - HSR uses RGB and depth image to find and localize known flat objects.')

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
        print(50 * '#' + '\n')
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
        print('\n' + 50*'#' + '\n')
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
        print('\n' + 50 * '#' + '\n')
        return 'succeeded'

class GoToNeutral(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
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
        else:
            rospy.loginfo('Could not return to neutral, is hsr connection established?')
        print('\n' + 50 * '#' + '\n')
        return 'succeeded'

class Quit(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Info')
        print('Abschalten')
        return 'succeeded'



def main():
    rospy.init_node('small_objects_statemachine')

    sm = smach.StateMachine(outcomes=['end'])

    # define some userdata
    sm.userdata.object_action = 'handover'
    sm.userdata.object_name = 'card'

    small_objects_sm = small_objects_statemachine.create_small_objects_sm()

    with sm:
        smach.StateMachine.add('USER_INPUT',
                               UserInput(),
                               transitions={'statemachine': 'small_objects_sm',
                                            'finding': 'Only_Find_Object',
                                            'picking': 'Only_Pick_Object',
                                            'laying': 'Only_Lay_Object',
                                            'start_suction': 'Start_Suction',
                                            'stop_suction': 'Stop_Suction',
                                            'neutral': 'Go_To_Neutral',
                                            'quit': 'end'},
                               remapping={'object_action': 'object_action',
                                          'object_name': 'object_name'})

        smach.StateMachine.add('small_objects_sm',
                               small_objects_sm,
                               transitions={'Exit': 'USER_INPUT'})

        smach.StateMachine.add('Only_Find_Object',
                               smach_ros.SimpleActionState('Find_Object_Action_Server', FindObjectAction,
                                                           goal_slots=['object_name']),
                               transitions={'succeeded': 'USER_INPUT',
                                            'preempted': 'USER_INPUT',
                                            'aborted': 'USER_INPUT'},
                               remapping={'object_name': 'object_name'})

        smach.StateMachine.add('Only_Pick_Object',
                               smach_ros.SimpleActionState('Pick_Object_Action_Server', PickObjectAction,
                                                           goal_slots=['object_name']),
                               transitions={'succeeded': 'USER_INPUT',
                                            'preempted': 'USER_INPUT',
                                            'aborted': 'USER_INPUT'},
                               remapping={'object_name': 'object_name'})

        smach.StateMachine.add('Only_Lay_Object',
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
