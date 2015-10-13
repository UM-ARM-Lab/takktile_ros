#!/usr/bin/env python
import rospy
from owd_msgs.msg import Servo
from sensor_msgs.msg import JointState

joint_names = ['/right/j1', '/right/j2', '/right/j3', '/right/j4', '/right/j5', '/right/j6', '/right/j7']

def servo_callback(input_msg):
    output_msg = Servo()

    for i, joint_name in enumerate(joint_names):
        output_msg.joint.append(i + 1)
        output_msg.velocity.append(input_msg.velocity[i])

    servo_pub.publish(output_msg)

rospy.init_node('servo_publisher')
servo_pub = rospy.Publisher('right/owd/wamservo', Servo, queue_size=1)
servo_sub = rospy.Subscriber('joints_servo', JointState, servo_callback)
rospy.spin()
