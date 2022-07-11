#!/usr/bin/env python
import rospy


import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion
from math import cos, sin


class FoundObjectBroadcaster:
    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        try:
            hsr_pos = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time(), rospy.Duration(6.0))
            # Test listener with shell: rosrun tf tf_echo base_link map
        except Exception as e:
            rospy.logerr('TF2 transform error:' + str(e))



        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            rospy.sleep(0.1)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = 'map'
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = 'found_object'

            coord_rot = euler_from_quaternion([hsr_pos.transform.rotation.x, hsr_pos.transform.rotation.y, hsr_pos.transform.rotation.z, hsr_pos.transform.rotation.w])

            # distance from hsr base_link to the object position
            object_dist_x = 1.0
            object_dist_y = 0
            object_dist_z = 0

            t.transform.translation.x = object_dist_x * cos(coord_rot[2]) - object_dist_y * sin(coord_rot[2]) + hsr_pos.transform.translation.x
            t.transform.translation.y = object_dist_x * sin(coord_rot[2]) + object_dist_y * cos(coord_rot[2]) + hsr_pos.transform.translation.y
            t.transform.translation.z = object_dist_z + hsr_pos.transform.translation.z

            t.transform.rotation.x = hsr_pos.transform.rotation.x
            t.transform.rotation.y = hsr_pos.transform.rotation.y
            t.transform.rotation.z = hsr_pos.transform.rotation.z
            t.transform.rotation.w = hsr_pos.transform.rotation.w

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

if __name__ == '__main__':
    rospy.init_node('found_object_broadcaster')
    tfb = FoundObjectBroadcaster()

    rospy.spin()