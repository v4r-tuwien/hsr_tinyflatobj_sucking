#!/usr/bin/env python3

import rospy
import pickle
from move_base_msgs.msg import MoveBaseGoal
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray


class Map:
    def __init__(self, name):
        self.map_name = name
        self.waypoints = []
        self.lay_down_point = None
        self.handover_point = None

    def add_waypoint(self, pos):
        waypoint = create_waypoint(pos[0], pos[1], pos[2], pos[3])
        self.waypoints.append(waypoint)

    def set_lay_down_point(self, pos):
        self.lay_down_point = create_waypoint(pos[0], pos[1], pos[2], pos[3])

    def set_handover_point(self, pos):
        self.handover_point = create_waypoint(pos[0], pos[1], pos[2], pos[3])

    def delete_points(self):
        self.waypoints = []
        self.lay_down_point = None
        self.handover_point = None

    def __str__(self):
        return self.map_name


class PointPublisher:
    def __init__(self):
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=2)

    def publish_points(self, map_class):
        # first delete all markers
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.action = marker.DELETEALL
        self.marker_pub.publish(marker)
        rospy.sleep(0.2)

        # displaying new markers
        if map_class.waypoints != []:
            for i in range(len(map_class.waypoints)):
                self.publish_point(map_class.waypoints[i], 'waypoint_' + str(i))
        if map_class.lay_down_point is not None:
            self.publish_point(map_class.lay_down_point, 'lay_down_point')
        if map_class.handover_point is not None:
            self.publish_point(map_class.handover_point, 'handover_point')

    def publish_point(self, waypoint, name):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.type = 0
        marker.id = 0
        marker.ns = name

        # Set scale
        marker.scale.x = 0.35
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        # Set the color
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        if name == 'lay_down_point':
            marker.color.g = 1.0
        elif name == 'handover_point':
            marker.color.b = 1.0
        else:
            marker.color.r = 1.0

        marker.pose.position.x = waypoint.target_pose.pose.position.x
        marker.pose.position.y = waypoint.target_pose.pose.position.y
        marker.pose.position.z = waypoint.target_pose.pose.position.z

        marker.pose.orientation.x = waypoint.target_pose.pose.orientation.x
        marker.pose.orientation.y = waypoint.target_pose.pose.orientation.y
        marker.pose.orientation.z = waypoint.target_pose.pose.orientation.z
        marker.pose.orientation.w = waypoint.target_pose.pose.orientation.w
        self.marker_pub.publish(marker)
        rospy.sleep(0.2)


def create_waypoint(x, y, q_z, q_w):
    move_goal = MoveBaseGoal()
    move_goal.target_pose.header.frame_id = 'map'
    move_goal.target_pose.pose.position.x = x
    move_goal.target_pose.pose.position.y = y
    move_goal.target_pose.pose.orientation.z = q_z
    move_goal.target_pose.pose.orientation.w = q_w
    return move_goal


class Position:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def get_robot_position(self):
        try:
            hsr_pos = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time(),
                                                  rospy.Duration(4.0))
            [x, y, q_z, q_w] = [hsr_pos.transform.translation.x, hsr_pos.transform.translation.y,
                                hsr_pos.transform.rotation.z, hsr_pos.transform.rotation.w]
        except Exception as e:
            rospy.loginfo('TF Error: ' + str(e))
            [x, y, q_z, q_w] = [0, 0, 0, 1]
        return [x, y, q_z, q_w]


def save_map(saved_map):
    with open('maps/' + str(saved_map) + '.pkl', 'wb') as f:
        pickle.dump(saved_map, f)


def create_and_load_maps():
    gazebo_tu_room = Map('gazebo_tu_room')
    gazebo_tu_room.add_waypoint([0, -1, -0.707, 0.707])
    gazebo_tu_room.add_waypoint([1, -1.3, -0.9489846, 0.3153224])
    gazebo_tu_room.set_handover_point([0, 0, 0, 1])
    gazebo_tu_room.set_lay_down_point([0.6, -0.4, 0, 1])
    save_map(gazebo_tu_room)

    tu_room = Map('tu_room')
    tu_room.add_waypoint([-0.3, 0.77, 0, 1])
    tu_room.add_waypoint([0.74, -0.11, 0.707, 0.707])
    tu_room.set_handover_point([0.46, -0.32, 0.7, 0.71])
    tu_room.set_lay_down_point([-0.2, -0.1, 0.8, -0.6])
    save_map(tu_room)

    # to create custom files
    #custom_1 = Map('custom_1')
    #custom_2 = Map('custom_2')
    #save_map(custom_1)
    #save_map(custom_2)

    try:
        with open('maps/custom_1.pkl', 'rb') as f:
            custom_1 = pickle.load(f)
        with open('maps/custom_2.pkl', 'rb') as f:
            custom_2 = pickle.load(f)
    except Exception as e:
        print('Error with custom map loading: ' + str(e))
        custom_1 = Map('custom_1')
        custom_2 = Map('custom_2')

    return gazebo_tu_room, tu_room, custom_1, custom_2

