#!/usr/bin/env python

import sys
import signal
import rospy

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from radariq_ros.msg import RadarIQObject, RadarIQObjects
from radariq import RadarIQ, MODE_OBJECT_TRACKING, OUTPUT_LIST

riq = None
colors = [[0, 0.5, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0.55, 0.27, 0.7],
          [0, 1, 1], [1, 0, 1], [0.39, 0.58, 0.93], [1, 0.41, 0.71], [1, 0.89, 0.77]]


def build_marker(detection):
    color_idx = detection['tracking_id'] % len(colors)

    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.ns = "riq-object-markers"
    marker.id = detection['tracking_id']
    marker.header.frame_id = "map"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.lifetime = rospy.Duration(1)  # Auto expire markers after 1 second unless they are updated
    marker.scale.x = 0.4
    marker.scale.y = 0.4
    marker.scale.z = 0.7
    marker.color.a = 1.0
    marker.color.r = colors[color_idx][0]
    marker.color.g = colors[color_idx][1]
    marker.color.b = colors[color_idx][2]
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = detection['x_pos']
    marker.pose.position.y = detection['y_pos']
    marker.pose.position.z = detection['z_pos']

    return marker


def build_marker_array(riq_objects):
    markers = MarkerArray()
    for detection in riq_objects:
        marker = build_marker(detection)
        markers.markers.append(marker)
    return markers


def build_object(detection):
    obj = RadarIQObject()
    obj.tracking_id = detection['tracking_id']
    obj.position.x = detection['x_pos']
    obj.position.y = detection['y_pos']
    obj.position.z = detection['z_pos']
    obj.velocity.x = detection['x_vel']
    obj.velocity.y = detection['y_vel']
    obj.velocity.z = detection['z_vel']
    obj.acceleration.x = detection['x_acc']
    obj.acceleration.y = detection['y_acc']
    obj.acceleration.z = detection['z_acc']
    return obj


def build_object_array(riq_objects):
    objs = RadarIQObjects()
    objs.header.stamp = rospy.Time.now()
    objs.header.frame_id = "map"
    for detection in riq_objects:
        obj = build_object(detection)
        objs.objects.append(obj)
    return objs


def run():
    global riq

    rospy.init_node("RadarIQObjectNode")
    serial_port = rospy.get_param('~serial_port')
    framerate = rospy.get_param('~framerate')
    distancefilter_min = rospy.get_param('~distancefilter_min')
    distancefilter_max = rospy.get_param('~distancefilter_max')
    anglefilter_min = rospy.get_param('~anglefilter_min')
    anglefilter_max = rospy.get_param('~anglefilter_max')
    pointdensity = rospy.get_param('~pointdensity')
    certainty = rospy.get_param('~certainty')
    marker_publisher = rospy.Publisher("/radariq_markers", MarkerArray, queue_size=10)
    object_publisher = rospy.Publisher("/radariq_objects", RadarIQObjects, queue_size=10)

    header = Header()
    header.frame_id = "map"

    try:
        riq = RadarIQ(port=serial_port, output_format=OUTPUT_LIST)
        riq.set_mode(MODE_OBJECT_TRACKING)
        riq.set_units('m', 'm/s')
        riq.set_frame_rate(framerate)
        riq.set_distance_filter(distancefilter_min, distancefilter_max)
        riq.set_angle_filter(anglefilter_min, anglefilter_max)
        riq.set_point_density(pointdensity)
        riq.set_certainty(certainty)
        riq.start()
        rospy.loginfo("Starting the RadarIQ module")

        for row in riq.get_data(100):
            if rospy.is_shutdown():
                break
            markers = build_marker_array(row)
            objs = build_object_array(row)
            marker_publisher.publish(markers)
            object_publisher.publish(objs)
            # rospy.loginfo("Input buffer length: {}".format(riq.get_queue_size()))

    except Exception as error:
        rospy.logerr(error)
    finally:
        del riq
        rospy.loginfo("Stopped RadarIQ module")


def signal_handler(sig, frame):
    global riq
    print('You pressed Ctrl+C!')
    riq.close()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    run()
