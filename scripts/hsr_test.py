#!/usr/bin/python
# -*- coding: utf-8 -*-

import hsrb_interface
import rospy
import sys
from hsrb_interface import geometry
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

# Move timeout[s]
_MOVE_TIMEOUT=60.0
# Grasp force[N]
_GRASP_FORCE=0.2


# Preparation for using the robot functions
robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')
gripper = robot.get('gripper')
tts = robot.get('default_tts')

# Posture that 0.02[m] front and rotate -1.57 around z-axis of the bottle maker
bottle_to_hand = geometry.pose(z=-0.02, ek=-1.57)

# Posture to move the hand 0.1[m] up
hand_up = geometry.pose(x=0.1)

# Posture to move the hand 0.5[m] back
hand_back = geometry.pose(z=-0.5)


if __name__=='__main__':


    try:
        #gripper.command(1.0)
        #whole_body.move_to_go()

        #whole_body.move_to_neutral()
        rospy.sleep(1.0)
        #whole_body.move_to_go()

    except:
        tts.say('Fail to initialize.')
        rospy.logerr('fail to init')
        sys.exit()

    try:
        # Move to the location where the bottle is viewable
        # omni_base.go_abs(sofa_pos[0], sofa_pos[1], sofa_pos[2], _MOVE_TIMEOUT)
        #whole_body.move_to_go()
        #rospy.sleep(2.0)

        #tts.say('Ready')
        rospy.logerr('Ready')

        #rospy.init_node('interesting_object_broadcaster')
        tfb = object_broadcaster()

        rospy.sleep(2.0)
        tts.say('Weiter')
        rospy.logerr('Weiter')

        rospy.spin()

        tts.say('Finished')
        rospy.logerr('Finished')

    except:
        tts.say('Fail to move.')
        rospy.logerr('fail to move')
        sys.exit()

    # try:
    #     # Transit to initial grasping posture
    #     whole_body.move_to_neutral()
    #     # Look at the hand after the transition
    #     whole_body.looking_hand_constraint = True
    #     # Move the hand to front of the bottle
    #     whole_body.move_end_effector_pose(bottle_to_hand, _BOTTLE_TF)
    #     # Specify the force to grasp
    #     gripper.apply_force(_GRASP_FORCE)
    #     # Wait time for simulator's grasp hack. Not needed on actual robot
    #     rospy.sleep(2.0)
    #     # Move the hand up on end effector coordinate
    #     whole_body.move_end_effector_pose(hand_up, _HAND_TF)
    #     # Move the hand back on end effector coordinate
    #     whole_body.move_end_effector_pose(hand_back, _HAND_TF)
    #     # Transit to initial posture
    #     whole_body.move_to_neutral()
    # except:
    #     tts.say('Fail to grasp.')
    #     rospy.logerr('fail to grasp')
    #     sys.exit()



class object_broadcaster:
     def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        while not rospy.is_shutdown():
             # Run this loop at about 10Hz
            rospy.sleep(0.1)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = 'head_rgbd_sensor_rgb_frame'
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = 'interesting_object'
            t.transform.translation.x = 0.0
            t.transform.translation.y = 2.0
            t.transform.translation.z = 0.0

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)