"""ROS2 node that subscribes to multiple topics and logs messages to CSV files."""

import csv
from datetime import datetime
from pathlib import Path
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rosidl_runtime_py.utilities import get_message


TOPICS_TO_SUBSCRIBE = [
    '/Laser_map',
    '/Odometry',
    '/cf_1/accgyro',
    '/cf_1/motors',
    '/cf_1/quaternions',
    '/cf_1/robot_description',
    '/clicked_point',
    '/cloud_effected',
    '/cloud_registered',
    '/cloud_registered_body',
    '/cmd_full_state',
    '/cmd_vel',
    '/cmd_vel_legacy',
    '/events/read_split',
    '/events/write_split',
    '/goal_pose',
    '/initialpose',
    '/joy',
    '/livox/accgyro',
    '/livox/imu',
    '/ompl_rrt_star_trajectory',
    '/parameter_events',
    '/path',
    '/rosout',
    '/rrt_star_grid',
    '/tf',
    '/trajectory_start_position',
]

SKIP_MSG_TYPES = [
    'sensor_msgs/msg/PointCloud2',
]


def flatten_message(msg, prefix=''):
    """Recursively flatten a ROS2 message to a dict with dot-notation keys."""
    result = {}

    if not hasattr(msg, 'get_fields_and_field_types'):
        return {prefix.rstrip('.'): msg}

    for field_name, field_type in msg.get_fields_and_field_types().items():
        value = getattr(msg, field_name)
        key = f"{prefix}{field_name}" if prefix else field_name

        if hasattr(value, 'get_fields_and_field_types'):
            result.update(flatten_message(value, f"{key}."))
        elif isinstance(value, (list, tuple)):
            if len(value) > 0 and hasattr(value[0], 'get_fields_and_field_types'):
                for i, item in enumerate(value):
                    result.update(flatten_message(item, f"{key}[{i}]."))
            else:
                result[key] = str(list(value))
        else:
            result[key] = value

    return result


def sanitize_topic_name(topic_name):
    """Convert topic name to valid filename."""
    name = topic_name.lstrip('/')
    name = name.replace('/', '_')
    return name


class CSVLoggerNode(Node):
    """Node that subscribes to topics and logs messages to CSV files."""

    def __init__(self, output_dir, discovery_period=2.0):
        super().__init__('csv_logger_node')
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.callback_group = ReentrantCallbackGroup()
        self.subscriptions_dict = {}
        self.csv_files = {}
        self.csv_writers = {}
        self.file_locks = {}
        self.headers_written = {}
        self.pending_topics = set(TOPICS_TO_SUBSCRIBE)

        self.get_logger().info(f'Output directory: {self.output_dir}')
        self.get_logger().info(f'Waiting for {len(self.pending_topics)} topics...')

        # Initial check
        self._discover_topics()

        # Periodic discovery timer (every 2 seconds by default)
        self.discovery_timer = self.create_timer(
            discovery_period,
            self._discover_topics,
        )

    def _discover_topics(self):
        """Check for new topics and subscribe to them."""
        if not self.pending_topics:
            return

        topic_types = dict(self.get_topic_names_and_types())
        newly_subscribed = []

        for topic_name in list(self.pending_topics):
            if topic_name not in topic_types:
                continue

            msg_type_str = topic_types[topic_name][0]

            if msg_type_str in SKIP_MSG_TYPES:
                self.get_logger().info(f'Skipping {topic_name} (type {msg_type_str})')
                self.pending_topics.discard(topic_name)
                continue

            try:
                msg_class = get_message(msg_type_str)
            except Exception as e:
                self.get_logger().error(f'Failed to get message class for {topic_name}: {e}')
                self.pending_topics.discard(topic_name)
                continue

            csv_filename = sanitize_topic_name(topic_name) + '.csv'
            csv_path = self.output_dir / csv_filename
            self.csv_files[topic_name] = open(csv_path, 'w', newline='')
            self.file_locks[topic_name] = Lock()
            self.headers_written[topic_name] = False

            sub = self.create_subscription(
                msg_class,
                topic_name,
                lambda msg, tn=topic_name: self._generic_callback(msg, tn),
                10,
                callback_group=self.callback_group,
            )
            self.subscriptions_dict[topic_name] = sub
            self.pending_topics.discard(topic_name)
            newly_subscribed.append(topic_name)
            self.get_logger().info(f'Subscribed to {topic_name} ({msg_type_str})')

        if newly_subscribed:
            self.get_logger().info(
                f'Now subscribed to {len(self.subscriptions_dict)} topics, '
                f'{len(self.pending_topics)} pending'
            )

    def _generic_callback(self, msg, topic_name):
        """Handle incoming messages by writing to CSV."""
        data = flatten_message(msg)
        data['_receive_timestamp_ns'] = self.get_clock().now().nanoseconds

        with self.file_locks[topic_name]:
            if not self.headers_written[topic_name]:
                self.csv_writers[topic_name] = csv.DictWriter(
                    self.csv_files[topic_name],
                    fieldnames=list(data.keys()),
                    extrasaction='ignore',
                )
                self.csv_writers[topic_name].writeheader()
                self.headers_written[topic_name] = True

            self.csv_writers[topic_name].writerow(data)
            self.csv_files[topic_name].flush()

    def shutdown(self):
        """Close all CSV files."""
        self.get_logger().info('Shutting down, closing CSV files...')
        for topic_name, f in self.csv_files.items():
            try:
                f.close()
                self.get_logger().info(f'Closed {topic_name}')
            except Exception as e:
                self.get_logger().error(f'Error closing {topic_name}: {e}')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=rclpy.SignalHandlerOptions.NO)

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    # Use fixed path to workspace csv_files folder
    output_dir = Path.home() / 'CrazySim' / 'data' / 'data_ballistic_planner' / 'hop_tracking' / timestamp

    node = CSVLoggerNode(output_dir)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Received Ctrl+C, shutting down...')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
