#!/usr/bin/env python
import numpy
import rospy
import time


class TakktileSubscriber(object):
    def __init__(self, input_topic, tare_topic, callback,
                 tare_count=100, filter_count=5):
        from collections import deque
        from threading import Condition, Lock

        self.lock = Lock()
        self.condition = Condition(self.lock)

        self.input_topic = input_topic
        self.tare_topic = tare_topic
        self.tare_count = tare_count
        self.filter_count = filter_count

        self.callback = callback
        self.queue = deque(maxlen=max(tare_count, filter_count))
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
            while len(self.queue) < self.tare_count:
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
            self.queue.append(takktile_msg)
            self.condition.notify_all()

            # Wait until we get enough data in the queue to filter.
            if len(self.queue) < self.filter_count:
                return
            # Wait until the sensor is tared.
            if self.tare_offset is None:
                return

            # Apply a low-pass filter to the data.
            pressure_filter_inputs = list(self.queue)[-self.filter_count:]
            pressure_filtered = numpy.mean(
                [numpy.array(msg.pressure, dtype='float')
                for msg in pressure_filter_inputs],
                axis=0)

            # Apply the tare offset.
            pressure_raw = numpy.array(takktile_msg.pressure)
            pressure_tared = pressure_filtered - self.tare_offset

        if pressure_tared is not None:
            self.callback(takktile_msg, pressure_tared)

    def tare_callback(self, request):
        from std_srvs.srv import TriggerResponse

        self.tare()

        return TriggerResponse(success=True, message='Tare complete.')


class TakktileBinarizer(object):
    def __init__(self, groups, callback, threshold=3.):
        self.group_map = list(groups)
        self.threshold = float(threshold)
        self.all_groups = set(groups)
        self.callback = callback

    def __call__(self, takktile_msg, pressure_tared):
        if len(self.group_map) != len(pressure_tared):
            rospy.logwarn(
                'Received unexpected number of pressure values: expected %d,'
                ' got %d.', len(self.group_map), len(pressure_tared))
            return

        pressure_mask = (pressure_tared > self.threshold)
        active_groups = set(
            group for group, is_active in zip(self.group_map, pressure_mask)
            if is_active)
        output = { group: group in active_groups for group in self.all_groups }

        return self.callback(takktile_msg, output)


class TakktileBinaryPublisher(object):
    def __init__(self, topic_name, queue_size=1):
        self.topic_name = topic_name
        self.queue_size = queue_size
        self.binary_pub = None

    def __enter__(self):
        from sensor_msgs.msg import JointState

        self.binary_pub = rospy.Publisher(
            self.topic_name, JointState, queue_size=self.queue_size)

    def __exit__(self, type, value, traceback):
        self.binary_pub.unregister()
        self.binary_pub = None

    def __call__(self, takktile_msg, sensor_map):
        from sensor_msgs.msg import JointState

        # FIXME: This is a potential race condition.
        if self.binary_pub is None:
            return

        output_msg = JointState()
        # TODO: This should actaully use the timestamp from the Takktile
        # message. However, the message currently has no header.
        output_msg.header.stamp = rospy.Time.now()
        output_msg.name = sensor_map.keys()
        output_msg.effort = [float(value) for value in sensor_map.itervalues()]

        self.binary_pub.publish(output_msg)


def main():
    import argparse
    import yaml
    from visualization_msgs.msg import MarkerArray

    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('--no-auto-tare', action='store_true',
        help='wait for an explicit tare service call before publishing')
    arg_parser.add_argument('groups_file', type=str,
        help='path to YAML description of sensor groups')
    args = arg_parser.parse_args() 

    with open(args.groups_file, 'r') as groups_file:
        groups_data = yaml.safe_load(groups_file)

    rospy.init_node('taktile_binary')

    #marker_pub = rospy.Publisher('takktile/markers', MarkerArray, queue_size=1)
    active_prev = set()

    def do_output(takktile_msg, sensor_map):
        binary_publisher(takktile_msg, sensor_map)

        active_curr = set(
            name for name, is_active in sensor_map.iteritems() if is_active)

        if active_curr != active_prev:
            rospy.loginfo('State changed: %s', ', '.join(sorted(active_curr)))

        active_prev.clear()
        active_prev.update(active_curr)

    binary_publisher = TakktileBinaryPublisher('takktile/groups')
    sensor_sub = TakktileSubscriber('takktile/raw', 'takktile/tare',
        TakktileBinarizer(groups_data, do_output))

    with binary_publisher, sensor_sub:
        if not args.no_auto_tare:
            sensor_sub.tare()

        rospy.spin()

    rospy.spin()

if __name__ == '__main__':
    main()
