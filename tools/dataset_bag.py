#!/usr/bin/env python3
"""Create ROS 2 bags from timestamped image/IMU CSVs, extract images, or convert ROS 1 bags."""
import argparse
import csv
from decimal import Decimal
import heapq
from pathlib import Path
import subprocess
import sys


def timestamp(value, unit):
    result = int(Decimal(value) * (1_000_000_000 if unit == 'seconds' else 1))
    if result < 0 or result >= (2**31)*1_000_000_000:
        raise ValueError('timestamp outside ROS message range')
    return result


def rows(path):
    previous = -1
    with Path(path).open(newline='') as stream:
        for row in csv.reader(line for line in stream if line.strip() and not line.lstrip().startswith('#')):
            try:
                Decimal(row[0])
            except Exception as error:
                raise ValueError(f'{path}: expected numeric timestamp, got {row[0]!r}') from error
            current = Decimal(row[0])
            if current <= previous:
                raise ValueError(f'{path}: timestamps must increase strictly')
            previous = current
            yield row


def create(args):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image, Imu
    from nav_msgs.msg import Odometry
    from PIL import Image as PILImage
    if not any([args.image_csv, args.imu_csv, args.groundtruth_csv]):
        raise ValueError('provide at least one input CSV')
    if args.output.exists():
        raise ValueError(f'output already exists: {args.output}')
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(args.output), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))

    def stream(path, topic, message_type, kind):
        writer.create_topic(rosbag2_py.TopicMetadata(id=0, name=topic, type=message_type, serialization_format='cdr'))
        for row in rows(path):
            t = timestamp(row[0], args.timestamp_unit)
            if kind == 'image':
                image_path = args.image_root / row[1]
                with PILImage.open(image_path) as image:
                    image = image.convert('L')
                    message = Image(height=image.height, width=image.width, encoding='mono8',
                                    is_bigendian=0, step=image.width, data=image.tobytes())
                message.header.frame_id = 'camera'
            elif kind == 'imu':
                if len(row) < 7: raise ValueError('IMU rows need timestamp,wx,wy,wz,ax,ay,az')
                message = Imu(); message.header.frame_id = 'body'
                message.orientation_covariance[0] = -1.
                message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = map(float, row[1:4])
                message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = map(float, row[4:7])
            else:
                if len(row) < 8: raise ValueError('ground-truth rows need timestamp,px,py,pz,qw,qx,qy,qz')
                message = Odometry(); message.header.frame_id = 'world'; message.child_frame_id = 'body'
                message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = map(float, row[1:4])
                message.pose.pose.orientation.w, message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z = map(float, row[4:8])
                if len(row) >= 11:
                    message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z = map(float, row[8:11])
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(t, 1_000_000_000)
            yield t, topic, serialize_message(message)

    streams = []
    if args.image_csv: streams.append(stream(args.image_csv, args.image_topic, 'sensor_msgs/msg/Image', 'image'))
    if args.imu_csv: streams.append(stream(args.imu_csv, args.imu_topic, 'sensor_msgs/msg/Imu', 'imu'))
    if args.groundtruth_csv: streams.append(stream(args.groundtruth_csv, args.groundtruth_topic, 'nav_msgs/msg/Odometry', 'groundtruth'))
    count = 0
    for t, topic, data in heapq.merge(*streams, key=lambda item: item[0]):
        writer.write(topic, data, t); count += 1
    del writer
    print(f'Wrote {count} messages to {args.output}')


def extract(args):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image
    from PIL import Image as PILImage
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.input), storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if types.get(args.topic) != 'sensor_msgs/msg/Image': raise ValueError('topic must contain sensor_msgs/msg/Image')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))
    args.output.mkdir(parents=True, exist_ok=False)
    count = 0
    with (args.output / 'data.csv').open('w', newline='') as stream:
        output = csv.writer(stream); output.writerow(['#timestamp [ns]', 'filename'])
        while reader.has_next():
            _, data, _ = reader.read_next()
            message = deserialize_message(data, Image)
            mode = {'mono8': 'L', 'rgb8': 'RGB', 'bgr8': 'RGB'}.get(message.encoding)
            if mode is None: raise ValueError(f'unsupported image encoding: {message.encoding}')
            raw = 'BGR' if message.encoding == 'bgr8' else mode
            image = PILImage.frombytes(mode, (message.width, message.height), bytes(message.data), 'raw', raw, message.step)
            t = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            name = f'{t}.png'; image.save(args.output / name); output.writerow([t, name]); count += 1
    print(f'Extracted {count} images')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    command = commands.add_parser('create')
    command.add_argument('--image-csv', type=Path)
    command.add_argument('--image-root', type=Path, default=Path('.'))
    command.add_argument('--imu-csv', type=Path)
    command.add_argument('--groundtruth-csv', type=Path)
    command.add_argument('--timestamp-unit', choices=['ns', 'seconds'], default='ns')
    command.add_argument('--image-topic', default='/cam0/image_raw')
    command.add_argument('--imu-topic', default='/imu0')
    command.add_argument('--groundtruth-topic', default='/groundtruth/odometry')
    command.add_argument('--output', type=Path, required=True)
    command.set_defaults(action=create)
    command = commands.add_parser('extract')
    command.add_argument('input', type=Path)
    command.add_argument('--topic', default='/cam0/image_raw')
    command.add_argument('--output', type=Path, required=True)
    command.set_defaults(action=extract)
    command = commands.add_parser('convert-ros1')
    command.add_argument('input', type=Path)
    command.add_argument('--output', type=Path, required=True)
    command.set_defaults(action=lambda args: subprocess.run([sys.executable, '-m', 'rosbags.convert', '--src', str(args.input), '--dst', str(args.output), '--dst-typestore', 'ros2_jazzy', '--dst-storage', 'mcap'], check=True))
    args = parser.parse_args()
    try: args.action(args)
    except (ValueError, OSError) as error: parser.error(str(error))


if __name__ == '__main__': main()
