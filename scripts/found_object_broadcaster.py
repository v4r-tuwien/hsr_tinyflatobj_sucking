#!/usr/bin/env python


import rospy
import tf2_ros
import tf2_msgs.msg
import tf2_geometry_msgs.tf2_geometry_msgs
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion
from math import cos, sin, isnan

from object_detector_msgs.msg import Detection as DetectronDetection
from object_detector_msgs.msg import Detections as DetectronDetections
from object_detector_msgs.srv import start, stop, detectron2_service_server, detectron2_service_serverResponse
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

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


def pointcloud_data(data):
    return data

def detectron_data(data):
    return data

if __name__ == '__main__':
    rospy.init_node('found_object_broadcaster')
    detectron_start = rospy.ServiceProxy('/detectron2_service/start', start)
    detectron_stop = rospy.ServiceProxy('/detectron2_service/stop', stop)

    pointcloud_topic = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points'
    pointcloud_sub = rospy.Subscriber(pointcloud_topic, PointCloud2, pointcloud_data)
    detectron_sub = rospy.Subscriber('/detectron2_service/detections', DetectronDetections, detectron_data)

    detectron_start()
    print('Waiting for detection')
    rospy.sleep(1)
    detections = rospy.wait_for_message('/detectron2_service/detections', DetectronDetections, timeout=10)
    detectron_stop()
    print('Detection received - stopping detectron')


    chosen_object = DetectronDetection()
    if len(detections.detections) != 0:
        for i in range(len(detections.detections)):
            name = detections.detections[i].name
            if name == 'apple':
                chosen_object = detections.detections[i]
                print('Object ' + name + ' detected')
                break
            else:
                chosen_object = -2

    else:
        chosen_object = -1
        print('Object not found')

    if chosen_object == -2:
        print('Specific object not found')
        print('Instead found:')
        for i in range(len(detections.detections)):
            name = detections.detections[i].name
            print(name)
        chosen_object = -1


    if chosen_object != -1:
        image_x = int(chosen_object.bbox.xmin +
                      (chosen_object.bbox.xmax - chosen_object.bbox.xmin) / 2)
        image_y = int(chosen_object.bbox.ymin +
                      (chosen_object.bbox.ymax - chosen_object.bbox.ymin) / 2)


        # get bounding box center from pointcloud
        cloud = rospy.wait_for_message(pointcloud_topic, PointCloud2, timeout=15)

        points = pc2.read_points_list(cloud, field_names=None, skip_nans=False)
        index = image_y * cloud.width + image_x
        center = points[index]

        # check if there is a valid point, otherwise go down a row
        while isnan(center[0]):
            index = index + cloud.width
            rospy.loginfo('index = {}'.format(index))
            center = points[index]

        tf_found_object = FoundObjectBroadcaster(center[0], center[1], center[2])
        print('Object position found')



    rospy.spin()