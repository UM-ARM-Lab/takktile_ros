#!/usr/bin/env python
import numpy
import rospy
import time


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

        return self

    def __exit__(self, type, value, traceback):
        self.takktile_sub.unregister()
        self.tare_srv.shutdown()

    def tare(self):
        rospy.loginfo('Starting tare.')
        time_before = time.time()

        with self.lock:
            # Wait for the queue to fill up.
            while len(self.queue) < self.queue.maxlen:
                self.condition.wait()

            # Compute the tare offset.
            pressures = numpy.array(
                [msg.pressure for msg in self.queue], dtype='float')
            self.tare_offset = numpy.mean(pressures, axis=0)

        time_after = time.time()
        rospy.loginfo('Completed tare after %.3f seconds.',
            time_after - time_before)

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

        self.tare()

        return TriggerResponse(success=True, message='Tare complete.')


class TakktileBinarizer(object):
    def __init__(self, groups, callback, threshold=4.):
        self.group_map = list(groups)
        self.threshold = float(threshold)
        self.all_groups = set(groups)
        self.callback = callback

    def binarize(self, takktile_msg, pressure_tared):
        if len(self.group_map) != len(pressure_tared):
            rospy.logwarn(
                'Received unexpected number of pressure values: expected %d,'
                ' got %d.', len(self.group_map), len(pressure_tared))
            return

        pressure_mask = (pressure_tared > self.threshold)
        active_groups = set(
            group for group, is_active in zip(self.group_map, pressure_mask)
            if is_active)
        print active_groups
        output = { group: group in active_groups for group in self.all_groups }

        return self.callback(takktile_msg, output)

def callback(takktile_msg, pressure):
    #print pressure
    pass

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

    thresholder = TakktileBinarizer(groups_data, callback)
    sensor_sub = TakktileSubscriber(
        'takktile/raw', 'takktile/tare', thresholder.binarize)

    with sensor_sub:
        sensor_sub.tare()
        rospy.spin()

    rospy.spin()

if __name__ == '__main__':
    main()
