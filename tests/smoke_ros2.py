#!/usr/bin/env python3
"""Exercise the installed ROS 2 pipeline with synthetic images and IMU (no dataset needed)."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = '217'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ.pop('ROS_LOCALHOST_ONLY', None)
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = rclpy.create_node('plvins_migration_smoke')
    counts = {'point': 0, 'line': 0, 'overlay': 0}
    def received(name):
        def callback(message):
            if name == 'overlay':
                assert message.encoding == 'bgr8' and message.width == 752 and message.height == 480
            else:
                assert len(message.channels) >= 3
                assert all(len(channel.values) == len(message.points) for channel in message.channels[:3])
            counts[name] += 1
        return callback
    subscriptions = [
        node.create_subscription(PointCloud, '/feature_tracker/feature', received('point'), 10),
        node.create_subscription(PointCloud, '/linefeature_tracker/linefeature', received('line'), 10),
        node.create_subscription(Image, '/PointLine_image', received('overlay'), 10),
    ]
    images = node.create_publisher(Image, '/cam0/image_raw', qos_profile_sensor_data)
    imus = node.create_publisher(Imu, '/imu0', qos_profile_sensor_data)
    save = node.create_client(Trigger, '/pose_graph/save_graph')
    process = None
    with tempfile.TemporaryDirectory(prefix='plvins-smoke-') as directory:
        log_path = Path(directory) / 'launch.log'
        with log_path.open('w') as log:
            process = subprocess.Popen(['ros2', 'launch', 'plvins_estimator', 'plvins.launch.py',
                'overlay:=true', 'rviz:=false', 'output_directory:=' + directory], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline and (images.get_subscription_count() < 3 or not save.service_is_ready()):
                if process.poll() is not None: raise RuntimeError(log_path.read_text())
                rclpy.spin_once(node, timeout_sec=.1)
            if not save.service_is_ready(): raise RuntimeError('Pose graph failed to start: ' + log_path.read_text())
            frame = np.full((480, 752), 255, dtype=np.uint8)
            frame[30:450:60, 20:730] = 0
            frame[20:460, 40:710:70] = 0
            # Thick deterministic edges produce both corner and line observations.
            frame = np.minimum(frame, np.roll(frame, 1, axis=0))
            frame = np.minimum(frame, np.roll(frame, 1, axis=1))
            for i in range(200):
                stamp = node.get_clock().now().to_msg()
                imu = Imu(); imu.header.stamp = stamp; imu.linear_acceleration.z = 9.81
                imus.publish(imu)
                if i % 5 == 0:
                    image = Image(height=480, width=752, encoding='mono8', step=752, data=frame.tobytes())
                    image.header.stamp = stamp; images.publish(image)
                rclpy.spin_once(node, timeout_sec=.01)
                time.sleep(.005)
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and not all(counts.values()): rclpy.spin_once(node, timeout_sec=.05)
            assert all(counts.values()), (counts, log_path.read_text())
            future = save.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            assert future.done() and future.result().success, log_path.read_text()
            assert (Path(directory) / 'pose_graph/pose_graph.txt').is_file()
            print('Pipeline smoke passed:', counts, '; pose-graph save succeeded')
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait()
                raise RuntimeError('Pipeline did not shut down cleanly: ' + log_path.read_text())
            log_text = log_path.read_text()
            if 'process has died' in log_text or process.returncode not in (0, 130):
                raise RuntimeError('Pipeline process failed: ' + log_text)
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
