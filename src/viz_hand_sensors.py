#!/usr/bin/env python

from __future__ import division
import rospy
import numpy
from takktile_ros.msg import Raw
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import ColorRGBA

LINE_WIDTH = 0.005

rospy.init_node('takktile_markers',anonymous=True)

from collections import namedtuple
TakkCell = namedtuple('TakkCell', ['frame_id', 'position', 'normal'])
# TODO figure out position values for all sensors
# TODO group by frame_id
# TODO load from file
giant_list = [
    TakkCell(frame_id='hand_base', position=numpy.array([0,0,0.1]), normal=numpy.array([0,0,1]))
] + [None]*83

SCALE_FACTOR = (.05/500)

def publish_viz(msg):
    """Publish RViz Markers for TakkTile sensors on Barrett Hand"""

    # TODO 1 marker per link, published as Marker[]
    ret_msg = Marker()
    ret_msg.header.frame_id = 'hand_base'
    # TODO add timestamp to takktile_ros.msg.Raw and use here
    ret_msg.header.stamp = rospy.Time.now()
    ret_msg.ns = 'takktile'
    # TODO different ID per marker
    ret_msg.id = 0
    ret_msg.action = Marker.MODIFY
    ret_msg.type = Marker.LINE_LIST
    ret_msg.pose.orientation.w = 1.0
    ret_msg.scale = Vector3(x=LINE_WIDTH,y=0,z=0)
    ret_msg.color = ColorRGBA(r=1,g=0,b=0,a=1)
    ret_msg.lifetime = rospy.Duration.from_sec(2/60)
    ret_msg.frame_locked = True

    for pres,tup in zip(msg.pressure,giant_list):
        if tup is not None:
            ret_msg.points.append(Point(*tup.position))
            p1 = tup.position + (tup.normal * pres * SCALE_FACTOR)
            ret_msg.points.append(Point(*p1))

    publisher.publish(ret_msg)


publisher = rospy.Publisher('takktile/markers', Marker, queue_size=1)

rospy.Subscriber('takktile/raw', Raw, publish_viz)
rospy.spin()
