#!/usr/bin/env python

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
from radariq import RIQ

topic = 'riq-markers'
publisher = rospy.Publisher(topic, MarkerArray)

rospy.init_node('register')


def build_marker(detection):
    marker = Marker()
    marker.header.stamp = ros::Time();  # todo
    marker.ns = "riq-object-markers"
    marker.id = detection.id
    marker.header.frame_id = "/base_link"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.lifetime = 1  # Auto expire markers after 1 second unless they are updated
    marker.scale.x = 0.2  # todo update the scales when size information comes through from the sdk
    marker.scale.y = 0.1
    marker.scale.z = 0.3
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = detection.x
    marker.pose.position.y = detection.y
    marker.pose.position.z = detection.z

    return marker


def build_marker_array(riq_objects):
    markers = MarkerArray()
    for detection in riq_objects:
        marker = build_marker(detection)
        markers.markers.append(marker)
    return markers


while data = riq.get_data():
    markers = build_marker_array(data)
    publisher.publish(markers)
