#!/usr/bin/env python

from __future__ import division
import argparse
import numpy
import rospy
import yaml
from collections import namedtuple
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from takktile_ros.msg import Raw
from visualization_msgs.msg import Marker, MarkerArray

# constants
LINE_WIDTH = 0.002 # 5mm
SCALE_FACTOR = (.05/500) # TODO calculate or allow as optional argument?
#MARKER_LIFETIME = rospy.Duration.from_sec(2/60)
MARKER_LIFETIME = rospy.Duration.from_sec(0.2) # TODO increased to eliminate flickering, fix flickering

# parse args and automatically open file for reading
arg_parser = argparse.ArgumentParser()
arg_parser.add_argument('sensor_file', type=argparse.FileType('r'),  help='file containing yaml description of sensor locations', metavar='filename')
args = arg_parser.parse_args() 

# load file
with args.sensor_file:
    sensor_data = yaml.safe_load(args.sensor_file)

# start node
rospy.init_node('takktile_markers',anonymous=True)

def publish_viz(data_msg):
    """Publish RViz Markers for TakkTile sensors relative to a set of base frame_id's"""

    pressure_vals = iter(data_msg.pressure)
    marker_array = []

    for link in sensor_data:
        # TODO figure out position values for all sensors
        for sensor in link['sensors']:
            try:
                pressure = pressure_vals.next()
                # marker metadata
                marker = Marker()
                marker.header.frame_id = link['frame_id']
                # TODO add timestamp to takktile_ros.msg.Raw and use here
                marker.header.stamp = rospy.Time.now()
                marker.ns = 'takktile'
                marker.id = len(marker_array) # must be unique per marker
                marker.action = Marker.MODIFY
                marker.type = Marker.LINE_LIST
                marker.pose.orientation.w = 1.0 # unit quaternion
                marker.scale = Vector3(x=LINE_WIDTH, y=0, z=0)
                marker.color = ColorRGBA(r=1, g=0, b=0, a=1)
                # 'takktile/raw' node publishes at 60hz, let markers fade if it stops publishing
                marker.lifetime = MARKER_LIFETIME
                marker.frame_locked = True # TODO verify this allows markers to move with moving frame

                # calculate and set endpoints of line
                marker.points.append(Point(*sensor['position'])) # position of sensor, start of line p0
                p1 = numpy.array(sensor['position']) + (numpy.array(sensor['normal']) * pressure * SCALE_FACTOR) # vector arith for line endpoint
                marker.points.append(Point(*p1))

                marker_array.append(marker)
            except StopIteration:
                print 'The number of sensors in the data file is more than the number of sensors being published' # TODO ros error log

    publisher.publish(marker_array)


publisher = rospy.Publisher('takktile/markers', MarkerArray, queue_size=1)

rospy.Subscriber('takktile/raw', Raw, publish_viz)
rospy.spin()
