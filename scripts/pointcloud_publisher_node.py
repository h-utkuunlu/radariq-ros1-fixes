#!/usr/bin/env python

# The MIT License
#
# Copyright (c) 2020 RadarIQ Limited https://radariq.io
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""
Publisher node for publishing controlling the RadarIQ module and publishing
to the /radariq ROS topic
"""

import sys
import signal
import rospy
from radariq import RadarIQ, MODE_POINT_CLOUD, OUTPUT_LIST

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

riq = None


def run():
    global riq

    rospy.init_node("RadarIQNode")
    serial_port = rospy.get_param('~serial_port')
    framerate = rospy.get_param('~framerate')
    distancefilter_min = rospy.get_param('~distancefilter_min')
    distancefilter_max = rospy.get_param('~distancefilter_max')
    anglefilter_min = rospy.get_param('~anglefilter_min')
    anglefilter_max = rospy.get_param('~anglefilter_max')
    pointdensity = rospy.get_param('~pointdensity')
    certainty = rospy.get_param('~certainty')
    print(serial_port)
    pub = rospy.Publisher("/radariq", PointCloud2, queue_size=10)

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('intensity', 16, PointField.FLOAT32, 1),
              ]

    header = Header()
    header.frame_id = "map"

    try:
        riq = RadarIQ(port=serial_port)
        riq.set_mode(MODE_POINT_CLOUD)
        riq.set_units('m', 'm/s')
        riq.set_frame_rate(framerate)
        riq.set_distance_filter(distancefilter_min, distancefilter_max)
        riq.set_angle_filter(anglefilter_min, anglefilter_max)
        riq.set_point_density(pointdensity)
        riq.set_certainty(certainty)
        riq.start()
        rospy.loginfo("Starting the RadarIQ module")

        for row in riq.get_data():
            if rospy.is_shutdown():
                break
            if row is not None:
                pc2 = point_cloud2.create_cloud(header, fields, row)
                pc2.header.stamp = rospy.Time.now()
                pub.publish(pc2)

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
