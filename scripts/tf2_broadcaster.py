#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import tf2_geometry_msgs.tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from math import cos, sin


class FoundObjectBroadcaster(object):
    def __init__(self, object_dist_x, object_dist_y, object_dist_z):
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        try:
            hsr_pos = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time(), rospy.Duration(6.0))
            # Test listener with shell: rosrun tf tf_echo base_link map
        except Exception as e:
            rospy.logerr('TF2 transform error:' + str(e))



        # object_dist = distance from hsr base_link to the object position
        depth_point = geometry_msgs.msg.PointStamped()
        depth_point.point.x = object_dist_x
        depth_point.point.y = object_dist_y
        depth_point.point.z = object_dist_z
        depth_point.header.frame_id = 'head_rgbd_sensor_rgb_frame'
        point_dist = self.tfBuffer.transform(depth_point, 'map', rospy.Duration(4.0))

        found_object_pos = geometry_msgs.msg.TransformStamped()
        found_object_pos.header.frame_id = 'map'
        found_object_pos.header.stamp = rospy.Time.now()
        found_object_pos.child_frame_id = 'found_object'

        # coord_rot = euler_from_quaternion([hsr_pos.transform.rotation.x, hsr_pos.transform.rotation.y, hsr_pos.transform.rotation.z, hsr_pos.transform.rotation.w])

        found_object_pos.transform.translation.x = point_dist.point.x
        found_object_pos.transform.translation.y = point_dist.point.y
        found_object_pos.transform.translation.z = point_dist.point.z

        found_object_pos.transform.rotation.x = hsr_pos.transform.rotation.x
        found_object_pos.transform.rotation.y = hsr_pos.transform.rotation.y
        found_object_pos.transform.rotation.z = hsr_pos.transform.rotation.z
        found_object_pos.transform.rotation.w = hsr_pos.transform.rotation.w

        #tfm = tf2_msgs.msg.TFMessage([t])
        self.broadcaster.sendTransform(found_object_pos)



            #print(point_transformed)

if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = FoundObjectBroadcaster(0,0,1)

    rospy.spin()