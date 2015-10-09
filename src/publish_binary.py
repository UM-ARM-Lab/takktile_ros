import argparse
import rospy
import yaml
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import MarkerArray
from takktile_ros.msg import Raw

def tare_callback(request):
    return TriggerResponse(success=False, message='Not implemented.')

def takktile_callback(takktile_msg):
    if len(takktile_msg.pressure) != len(groups_data):
        rospy.logwarn(
            'takktile_ros/Raw message contains %d pressure values, but there'
            ' there are groups defined for %d values.',
            len(takktile_msg.pressure), len(groups_data))
        return

    pass

def main():
    global sensor_data, groups_data, marker_pub

    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('sensor_file', type=str,
        help='path to YAML description of sensor locations')
    arg_parser.add_argument('groups_file', type=str,
        help='path to YAML description of sensor groups')
    args = arg_parser.parse_args() 

    with open(args.sensor_file, 'r') as sensor_file:
        sensor_data = yaml.safe_load(sensor_file)

    with open(args.groups_file, 'r') as groups_file:
        groups_data = yaml.safe_load(groups_file)

    rospy.init_node('taktile_binary')
        
    marker_pub = rospy.Publisher('takktile/markers', MarkerArray, queue_size=1)
    takktile_sub = rospy.Subscriber('takktile/raw', Raw, takktile_callback)
    tare_srv = rospy.Service('takktile/tare', Trigger, tare_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
