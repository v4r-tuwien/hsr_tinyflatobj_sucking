#!/usr/bin/env python


import rospy
import tf2_ros
import tf2_msgs.msg
import tf2_geometry_msgs.tf2_geometry_msgs
import geometry_msgs.msg
from math import cos, sin, isnan

from object_detector_msgs.msg import Detection as DetectronDetection
from object_detector_msgs.msg import Detections as DetectronDetections
from object_detector_msgs.srv import start, stop, detectron2_service_server, detectron2_service_serverResponse
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from hsr_small_objects.msg import FindObjectAction, FindObjectActionResult
import actionlib
import roslib
roslib.load_manifest('hsr_small_objects')

class FindObjectActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('Find_Object_Action_Server', FindObjectAction, self.execute, False)
        self.server.start()

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.detectron_start = rospy.ServiceProxy('/detectron2_service/start', start)
        self.detectron_stop = rospy.ServiceProxy('/detectron2_service/stop', stop)

        rospy.loginfo('Find_object_action_server init finished')

    def execute(self, goal):
        # define result of action_sever
        result = FindObjectActionResult().result

        try:
            self.detectron_start()
            rospy.loginfo('Detectron started - waiting for detection')
            rospy.sleep(1)
            detections = rospy.wait_for_message('/detectron2_service/detections', DetectronDetections, timeout=20)
            self.detectron_stop()
            rospy.loginfo('Detection received - stopping detectron')
        except Exception as e:
            rospy.loginfo('Error with detectron occurred: ' + str(e))
            result.result_info = 'Detectron error'
            self.server.set_succeeded(result)
            return

        chosen_object = DetectronDetection()
        if len(detections.detections) != 0:
            for i in range(len(detections.detections)):
                name = detections.detections[i].name
                if name == goal.object_name:
                    chosen_object = detections.detections[i]
                    print('Object ' + name + ' detected')
                    break
                else:
                    chosen_object = -2

        else:
            rospy.loginfo('No object found')
            result.result_info = 'not_found'
            self.server.set_succeeded(result)
            return

        if chosen_object == -2:
            rospy.loginfo('Specific object not found')
            rospy.loginfo('Instead found:')
            for i in range(len(detections.detections)):
                name = detections.detections[i].name
                rospy.loginfo(name)
            result.result_info = 'not_found'
            self.server.set_succeeded(result)
            return

        if chosen_object != -1:
            image_x = int(chosen_object.bbox.xmin +
                          (chosen_object.bbox.xmax - chosen_object.bbox.xmin) / 2)
            image_y = int(chosen_object.bbox.ymin +
                          (chosen_object.bbox.ymax - chosen_object.bbox.ymin) / 2)

            # get bounding box center from pointcloud
            pointcloud_topic = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points'
            cloud = rospy.wait_for_message(pointcloud_topic, PointCloud2, timeout=15)

            points = pc2.read_points_list(cloud, field_names=None, skip_nans=False)
            index = image_y * cloud.width + image_x
            center = points[index]

            # check if there is a valid point, otherwise go down a row
            while isnan(center[0]):
                index = index + cloud.width
                rospy.loginfo('index = {}'.format(index))
                center = points[index]

            try:
                hsr_pos = self.tfBuffer.lookup_transform('odom', 'base_link', rospy.Time(), rospy.Duration(6.0))
            except Exception as e:
                rospy.loginfo('TF2 transform error:' + str(e))
                result.result_info = 'robot_problem'
                self.server.set_succeeded(result)
                return

            # object_dist = distance from hsr base_link to the object position
            depth_point = geometry_msgs.msg.PointStamped()
            depth_point.point.x = center[0]
            depth_point.point.y = center[1]
            depth_point.point.z = center[2]
            depth_point.header.frame_id = 'head_rgbd_sensor_rgb_frame'
            point_dist = self.tfBuffer.transform(depth_point, 'odom', rospy.Duration(4.0))

            found_object_pos = geometry_msgs.msg.PoseStamped()
            found_object_pos.header.frame_id = 'odom'
            found_object_pos.header.stamp = rospy.Time.now()

            found_object_pos.pose.position.x = point_dist.point.x
            found_object_pos.pose.position.y = point_dist.point.y
            found_object_pos.pose.position.z = point_dist.point.z

            found_object_pos.pose.orientation.x = hsr_pos.transform.rotation.x
            found_object_pos.pose.orientation.y = hsr_pos.transform.rotation.y
            found_object_pos.pose.orientation.z = hsr_pos.transform.rotation.z
            found_object_pos.pose.orientation.w = hsr_pos.transform.rotation.w

            self.publish_frame(found_object_pos, goal.object_name)
            result.result_info = 'succeeded'
            self.server.set_succeeded(result)

    def publish_frame(self, pose_stamped, frame_name):
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()

        goal_frame = geometry_msgs.msg.TransformStamped()
        goal_frame.header.frame_id = 'odom'
        goal_frame.header.stamp = rospy.Time.now()

        goal_frame.child_frame_id = 'found_object_' + frame_name
        goal_frame.transform.translation.x = pose_stamped.pose.position.x
        goal_frame.transform.translation.y = pose_stamped.pose.position.y
        goal_frame.transform.translation.z = pose_stamped.pose.position.z

        goal_frame.transform.rotation.x = pose_stamped.pose.orientation.x
        goal_frame.transform.rotation.y = pose_stamped.pose.orientation.y
        goal_frame.transform.rotation.z = pose_stamped.pose.orientation.z
        goal_frame.transform.rotation.w = pose_stamped.pose.orientation.w

        self.broadcaster.sendTransform(goal_frame)


if __name__ == '__main__':

    rospy.init_node('find_object_action_server')
    server = FindObjectActionServer()
    print('Find_object_action_server started')
    rospy.spin()
