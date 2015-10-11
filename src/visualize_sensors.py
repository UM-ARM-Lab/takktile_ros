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
SCALE_FACTOR = (.15/500) # TODO calculate or allow as optional argument?
MARKER_LIFETIME = rospy.Duration.from_sec(2/60)

# parse args and automatically open file for reading
arg_parser = argparse.ArgumentParser()
arg_parser.add_argument('sensor_file', type=argparse.FileType('r'),  help='file containing yaml description of sensor locations', metavar='filename')
# TODO add arg to invert marker direction
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
        for sensor in link['sensors']:
            try:
                pressure = pressure_vals.next()
                # marker metadata
                marker = Marker()
                marker.header.frame_id = link['frame_id']
                marker.header.seq = data_msg.header.seq
                marker.header.stamp = data_msg.header.stamp
                marker.ns = 'takktile'
                marker.id = len(marker_array) # must be unique per marker TODO better/faster/stronger?
                marker.action = Marker.MODIFY
                marker.type = Marker.LINE_LIST
                marker.pose.orientation.w = 1.0 # unit quaternion
                marker.scale = Vector3(x=LINE_WIDTH, y=0, z=0)
                marker.color = ColorRGBA(r=1, g=0, b=0, a=1)
                # 'takktile/raw' node publishes at 60hz, let markers fade if it stops publishing
                marker.lifetime = MARKER_LIFETIME
                marker.frame_locked = True

                # calculate and set endpoints of line
                marker.points.append(Point(*sensor['position'])) # position of sensor, start of line p0
                # raw direction
                p1 = numpy.array(sensor['position']) + (numpy.array(sensor['normal']) * pressure * SCALE_FACTOR) # vector arith for line endpoint
                # inverted
                # p1 = numpy.array(sensor['position']) + (numpy.array(sensor['normal']) * (300 - pressure) * SCALE_FACTOR) # vector arith for line endpoint
                # TODO make inverted not go negative into hand
                marker.points.append(Point(*p1))

                marker_array.append(marker)
            except StopIteration:
                rospy.logwarn('The number of sensors in the data file is more than the number of sensors being published')

    publisher.publish(marker_array)


publisher = rospy.Publisher('takktile/markers', MarkerArray, queue_size=1)

rospy.Subscriber('takktile/raw', Raw, publish_viz)
rospy.spin()
