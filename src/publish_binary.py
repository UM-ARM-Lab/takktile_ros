#!/usr/bin/env python
import numpy
import rospy


class TakktileSubscriber(object):
    def __init__(self, input_topic, tare_topic, callback, count=100):
        from collections import deque
        from threading import Condition, Lock

        self.lock = Lock()
        self.condition = Condition(self.lock)

        self.input_topic = input_topic
        self.tare_topic = tare_topic

        self.callback = callback
        self.queue = deque(maxlen=count)
        self.tare_offset = None

    def __enter__(self):
        from std_srvs.srv import Trigger
        from takktile_ros.msg import Raw

        self.takktile_sub = rospy.Subscriber(
            self.input_topic, Raw, self.data_callback)
        self.tare_srv = rospy.Service(
            self.tare_topic, Trigger, self.tare_callback)

    def __exit__(self, type, value, traceback):
        self.takktile_sub.unregister()
        self.tare_srv.shutdown()

    def data_callback(self, takktile_msg):
        with self.lock:
            # Accumulate a buffer of data to use for tareing.
            self.queue.append(takktile_msg)
            self.condition.notify_all()

            # Apply the tare offset.
            if self.tare_offset is not None:
                pressure_raw = numpy.array(takktile_msg.pressure)
                pressure_tared = pressure_raw - self.tare_offset
            else:
                pressure_tared = None

        if pressure_tared is not None:
            self.callback(takktile_msg, pressure_tared)

    def tare_callback(self, request):
        from std_srvs.srv import TriggerResponse

        with self.lock:
            # Wait for the queue to fill up.
            while len(self.queue) < self.queue.maxlen:
                self.condition.wait()

            # Compute the tare offset.
            pressures = numpy.array(
                [msg.pressure for msg in self.queue], dtype='float')
            self.tare_offset = numpy.mean(pressures, axis=0)

        return TriggerResponse(success=True, message='Tare complete.')

def callback(takktile_msg, pressure):
    print pressure

def main():
    import argparse
    import yaml
    from visualization_msgs.msg import MarkerArray

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

    with TakktileSubscriber('takktile/raw', 'takktile/tare', callback):
        rospy.spin()

    rospy.spin()

if __name__ == '__main__':
    main()
