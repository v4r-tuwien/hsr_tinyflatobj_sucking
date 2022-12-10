#!/usr/bin/env python3

from enum import Enum
from math import pi

import actionlib
import rospy
import smach
import smach_ros

#from actionlib_msgs.msg import GoalStatus

#from handover.msg import HandoverAction
from hsrb_interface import Robot, geometry
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import roslib
roslib.load_manifest('hsr_small_objects')
from hsr_small_objects.msg import FindObjectAction, ArmMovementAction, ArmMovementGoal
from handover.msg import HandoverAction
import small_objects_statemachine
import waypoint_maps
import pickle

# neutral joint positions
neutral_joint_positions = {'arm_flex_joint': 0.0,
                           'arm_lift_joint': 0.0,
                           'arm_roll_joint': -1.570,
                           'hand_motor_joint': 0.3,
                           'head_pan_joint': 0.0,
                           'head_tilt_joint': -0.5,
                           'wrist_flex_joint': -1.57,
                           'wrist_roll_joint': 0.0}


# Enum for states
class States(Enum):
    START_STATEMACHINE = 1
    FIND_OBJECT = 2
    PICK_UP = 3
    LAY_DOWN = 4
    GAZE_AT_POINT = 5
    ACTIVATE_SUCTION = 6
    DEACTIVATE_SUCTION = 7
    RETURN_TO_NEUTRAL = 8
    HANDOVER = 9
    MAP_SETTINGS = 10
    SETTINGS = 11
    INFO = 12
    QUIT = 13


# Mapping of states to characters
states_keys = {States.START_STATEMACHINE: 's',
               States.FIND_OBJECT: 'f',
               States.PICK_UP: 'p',
               States.LAY_DOWN: 'l',
               States.GAZE_AT_POINT: 'g',
               States.ACTIVATE_SUCTION: 'a',
               States.DEACTIVATE_SUCTION: 'd',
               States.RETURN_TO_NEUTRAL: 'n',
               States.HANDOVER: 'h',
               States.MAP_SETTINGS: 'm',
               States.SETTINGS: 'c',
               States.INFO: 'i',
               States.QUIT: 'q'}

# Load waypoints
[gazebo_tu_room, tu_room, custom_1, custom_2] = waypoint_maps.create_and_load_maps()


# To save settings as class
class Settings:
    def __init__(self):
        self.object_action = 'handover'
        self.object_name = 'card'
        self.map = 'tu_room'
        self.gaze_height = 0.5


# Load or create settings
try:
    with open('maps/settings.pkl', 'rb') as f:
        settings = pickle.load(f)
except Exception as e:
    print('Error with settings loading: ' + str(e))
    settings = Settings()
    with open('maps/settings.pkl', 'wb') as f:
        pickle.dump(settings, f)


class UserInput(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['statemachine', 'finding', 'picking', 'laying', 'gaze', 'start_suction',
                                             'stop_suction', 'neutral', 'handover', 'quit'],
                             input_keys=['object_action', 'object_name', 'map', 'gaze_height', 'waypoint_number'],
                             output_keys=['object_action', 'object_name', 'map', 'gaze_height', 'waypoint_number'])

        self.hsr_position = waypoint_maps.Position()  # for adding waypoints
        self.point_publisher = waypoint_maps.PointPublisher()  # for publishing points

    def execute(self, userdata):
        rospy.loginfo('Executing state UserInput')

        while not rospy.is_shutdown():
            self.print_info()

            while True:
                user_input = input('\nCMD> ')
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

            elif char_in == states_keys[States.GAZE_AT_POINT]:
                print('Gaze at point 1 meter away at height: ' + str(userdata.gaze_height))
                return 'gaze'

            elif char_in == states_keys[States.ACTIVATE_SUCTION]:
                print('Starting suction cup')
                return 'start_suction'

            elif char_in == states_keys[States.DEACTIVATE_SUCTION]:
                print('Stopping suction cup')
                return 'stop_suction'

            elif char_in == states_keys[States.RETURN_TO_NEUTRAL]:
                print('Returning to neutral position')
                return 'neutral'

            elif char_in == states_keys[States.HANDOVER]:
                print('Going to handover position')
                return 'handover'

            elif char_in == states_keys[States.MAP_SETTINGS]:
                while not rospy.is_shutdown():
                    print('\nMap Settings:\n')
                    print('1 - Change map: \t' + str(userdata.map))
                    print('2 - Add position as waypoint')
                    print('3 - Set position as handover_point')
                    print('4 - Set position as lay_down_point')
                    print('5 - Save points')
                    print('6 - Publish point markers')
                    print('7 - Delete all waypoints for map')
                    print('')
                    print('8 - Back')
                    while True:
                        user_input = input('\nCMD> ')
                        if len(user_input) == 1:
                            break
                        print('Please enter only one character')
                    char_in = user_input.lower()
                    if char_in == '1':
                        if userdata.map == gazebo_tu_room:
                            userdata.map = tu_room
                        elif userdata.map == tu_room:
                            userdata.map = custom_1
                        elif userdata.map == custom_1:
                            userdata.map = custom_2
                        else:
                            userdata.map = gazebo_tu_room
                    elif char_in == '2':
                        pos = self.hsr_position.get_robot_position()
                        userdata.map.add_waypoint(pos)
                        print('Position: ' + str(pos))
                        print('was added as waypoint')
                    elif char_in == '3':
                        pos = self.hsr_position.get_robot_position()
                        userdata.map.set_handover_point(pos)
                        print('Position: ' + str(pos))
                        print('was added as handover_point')
                    elif char_in == '4':
                        pos = self.hsr_position.get_robot_position()
                        userdata.map.set_lay_down_point(pos)
                        print('Position: ' + str(pos))
                        print('was added as lay_down_point')
                    elif char_in == '5':
                        waypoint_maps.save_map(userdata.map)
                        print('Map: ' + str(userdata.map) + ' was saved')
                    elif char_in == '6':
                        self.point_publisher.publish_points(userdata.map)
                        print('Publishing points as markers')
                        print('Red - Waypoint / Green - Lay_Down_Point / Blue - Handover_Point')
                    elif char_in == '7':
                        userdata.map.delete_points()
                        print('Points deleted')

                    elif char_in == '8':
                        break
                    else:
                        print('No valid option.')

            elif char_in == states_keys[States.SETTINGS]:
                while not rospy.is_shutdown():
                    print('\nSettings:\n')
                    print('1 - After pick-up:\t' + str(userdata.object_action))
                    print('2 - Find object:\t' + str(userdata.object_name))
                    print('3 - Waypoint map:\t' + str(userdata.map))
                    print('4 - Gaze point height:\t' + str(userdata.gaze_height))
                    print('5 - Reset waypoint number')
                    print('')
                    print('6 - Back')
                    while True:
                        user_input = input('\nCMD> ')
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
                            userdata.object_name = 'box'
                        elif userdata.object_name == 'box':
                            userdata.object_name = 'coin'
                        else:
                            userdata.object_name = 'card'
                    elif char_in == '3':
                        if userdata.map == gazebo_tu_room:
                            userdata.map = tu_room
                        elif userdata.map == tu_room:
                            userdata.map = custom_1
                        elif userdata.map == custom_1:
                            userdata.map = custom_2
                        else:
                            userdata.map = gazebo_tu_room
                    elif char_in == '4':
                        if userdata.gaze_height == 0.5:
                            userdata.gaze_height = 0.7
                        elif userdata.gaze_height == 0.7:
                            userdata.gaze_height = 0.3
                        else:
                            userdata.gaze_height = 0.5
                    elif char_in == '5':
                        userdata.waypoint_number = 0
                    elif char_in == '6':
                        break
                    else:
                        print('No valid option.')

                    # Save settings
                    settings.object_action = userdata.object_action
                    settings.object_name = userdata.object_name
                    settings.map = str(userdata.map)
                    with open('maps/settings.pkl', 'wb') as f:
                        pickle.dump(settings, f)

            elif char_in is None or char_in == states_keys[States.INFO]:
                print('\n\n\tFind_Object - HSR uses RGB and depth image to find and localize known flat objects.')

            elif char_in is None or char_in == states_keys[States.QUIT]:
                print('\nTschau')
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


class GazeAtPoint(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['gaze_height'])
        self.robot = Robot()
        try:
            self.whole_body = self.robot.try_get('whole_body')
        except Exception as e:
            rospy.loginfo('Problem finding: ' + str(e))
            self.whole_body = None

    def execute(self, userdata):
        print(50*'#' + '\n')
        if self.whole_body is not None:
            rospy.loginfo('Gazing at point')
            self.whole_body.gaze_point(point=geometry.Vector3(x=1.0, y=0.0, z=userdata.gaze_height), ref_frame_id='base_link')
        else:
            rospy.loginfo('Could not gaze at point, is hsr connection established?')
        print('\n' + 50*'#' + '\n')
        return 'succeeded'


class StartSuction(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot = Robot()
        self.suction = self.robot.try_get('suction')

    def execute(self, userdata):
        print(50*'#' + '\n')
        if self.suction is not None:
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
        if self.suction is not None:
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
        if self.whole_body is not None:
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


def main():
    rospy.init_node('small_objects_statemachine')

    sm = smach.StateMachine(outcomes=['end'])

    # define some userdata from settings
    sm.userdata.object_action = settings.object_action
    sm.userdata.object_name = settings.object_name
    sm.userdata.gaze_height = settings.gaze_height
    sm.userdata.waypoint_number = 0
    for i in [gazebo_tu_room, tu_room, custom_1, custom_2]:
        if str(i) == settings.map:
            sm.userdata.map = i

    small_objects_sm = small_objects_statemachine.create_small_objects_sm()

    with sm:
        smach.StateMachine.add('USER_INPUT',
                               UserInput(),
                               transitions={'statemachine': 'small_objects_sm',
                                            'finding': 'Only_Find_Object',
                                            'picking': 'Only_Pick_Object',
                                            'laying': 'Only_Lay_Object',
                                            'gaze': 'Gaze_At_Point',
                                            'start_suction': 'Start_Suction',
                                            'stop_suction': 'Stop_Suction',
                                            'neutral': 'Go_To_Neutral',
                                            'handover': 'Handover',
                                            'quit': 'end'},
                               remapping={'object_action': 'object_action',
                                          'object_name': 'object_name',
                                          'map': 'map',
                                          'gaze_height': 'gaze_height'})

        smach.StateMachine.add('small_objects_sm',
                               small_objects_sm,
                               transitions={'Exit': 'USER_INPUT',
                                            'Exit_successful': 'USER_INPUT'},
                               remapping={'map': 'map',
                                          'object_action': 'object_action',
                                          'object_name': 'object_name',
                                          'waypoint_number': 'waypoint_number'})

        smach.StateMachine.add('Only_Find_Object',
                               smach_ros.SimpleActionState('Find_Object_Action_Server', FindObjectAction,
                                                           goal_slots=['object_name'],
                                                           exec_timeout=rospy.Duration(35.0),
                                                           preempt_timeout=rospy.Duration(2.0),
                                                           server_wait_timeout=rospy.Duration(25.0)),
                               transitions={'succeeded': 'USER_INPUT',
                                            'preempted': 'USER_INPUT',
                                            'aborted': 'USER_INPUT'},
                               remapping={'object_name': 'object_name'})

        smach.StateMachine.add('Only_Pick_Object',
                               smach_ros.SimpleActionState('Arm_Movement_Action_Server', ArmMovementAction,
                                                           goal_slots=['command'],
                                                           exec_timeout=rospy.Duration(50.0),
                                                           preempt_timeout=rospy.Duration(2.0),
                                                           server_wait_timeout=rospy.Duration(25.0)),
                               transitions={'succeeded': 'USER_INPUT',
                                            'preempted': 'USER_INPUT',
                                            'aborted': 'USER_INPUT'},
                               remapping={'command': 'object_name'})

        smach.StateMachine.add('Only_Lay_Object',
                               smach_ros.SimpleActionState('Arm_Movement_Action_Server', ArmMovementAction,
                                                           goal=ArmMovementGoal('lay_down'),
                                                           exec_timeout=rospy.Duration(35.0),
                                                           preempt_timeout=rospy.Duration(2.0),
                                                           server_wait_timeout=rospy.Duration(25.0)),
                               transitions={'succeeded': 'USER_INPUT',
                                            'preempted': 'USER_INPUT',
                                            'aborted': 'USER_INPUT'})

        smach.StateMachine.add('Gaze_At_Point',
                               GazeAtPoint(),
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

        smach.StateMachine.add('Handover', smach_ros.SimpleActionState('/handover', HandoverAction,
                                                                       exec_timeout=rospy.Duration(25.0),
                                                                       preempt_timeout=rospy.Duration(2.0),
                                                                       server_wait_timeout=rospy.Duration(25.0)),
                               transitions={'succeeded': 'USER_INPUT',
                                            'preempted': 'USER_INPUT',
                                            'aborted': 'USER_INPUT'})



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
