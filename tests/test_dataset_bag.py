import importlib.util
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('dataset_bag', ROOT / 'tools/dataset_bag.py')
bag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bag)


class DatasetTools(unittest.TestCase):
    def test_timestamp_precision(self):
        self.assertEqual(bag.timestamp('1403636579.763555584', 'seconds'), 1403636579763555584)
        self.assertEqual(bag.timestamp('1403636579763555584', 'ns'), 1403636579763555584)
        with self.assertRaises(ValueError): bag.timestamp('-1', 'ns')

    def test_reject_unordered_csv(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'imu.csv'
            path.write_text('2,0\n1,0\n')
            with self.assertRaises(ValueError): list(bag.rows(path))

    def test_image_imu_bag_roundtrip(self):
        import numpy as np
        from PIL import Image
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Image as ImageMessage
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pixels = np.arange(256, dtype=np.uint8).reshape(16, 16)
            Image.fromarray(pixels).save(root / 'frame.png')
            (root / 'images.csv').write_text('#timestamp,filename\n1000000001,frame.png\n')
            (root / 'imu.csv').write_text('#timestamp,wx,wy,wz,ax,ay,az\n1000000000,0,0,0,0,0,9.81\n1000000002,0,0,0,0,0,9.81\n')
            bag.create(SimpleNamespace(output=root / 'bag', image_csv=root / 'images.csv', image_root=root,
                imu_csv=root / 'imu.csv', groundtruth_csv=None, timestamp_unit='ns',
                image_topic='/cam0/image_raw', imu_topic='/imu0', groundtruth_topic='/gt'))
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(root / 'bag'), storage_id=''), rosbag2_py.ConverterOptions('', ''))
            times = []
            while reader.has_next():
                topic, data, t = reader.read_next(); times.append(t)
                if topic == '/cam0/image_raw':
                    message = deserialize_message(data, ImageMessage)
                    self.assertEqual(message.header.stamp.sec, 1)
                    self.assertEqual(message.header.stamp.nanosec, 1)
                    self.assertEqual(bytes(message.data), pixels.tobytes())
            self.assertEqual(times, [1000000000, 1000000001, 1000000002])
            bag.extract(SimpleNamespace(input=root / 'bag', output=root / 'extracted', topic='/cam0/image_raw'))
            np.testing.assert_array_equal(np.array(Image.open(root / 'extracted/1000000001.png')), pixels)


if __name__ == '__main__': unittest.main()
