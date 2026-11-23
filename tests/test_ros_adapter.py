"""Binary transport checks; also run inside ROS with python3 -m unittest."""
import base64
import unittest

try:
    from sensor_msgs.msg import Image, CompressedImage, PointCloud2, CameraInfo
    from std_msgs.msg import String
    from urc_bringup.node import from_ros, to_ros
    from urc_core.runtime import as_bytes
    ROS_AVAILABLE = True
except ModuleNotFoundError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, 'requires the ROS 2 workspace')
class BinaryTransportTests(unittest.TestCase):
    def test_depth_pixels_and_byte_order_survive_input_conversion(self):
        for big_endian in (False, True):
            with self.subTest(big_endian=big_endian):
                order = 'big' if big_endian else 'little'
                pixels = b''.join(value.to_bytes(2, order) for value in (0, 470, 1234, 65535)) * 57600
                message = Image(height=360, width=640, encoding='16UC1',
                                is_bigendian=big_endian, step=1280, data=pixels)
                message.header.frame_id = 'camera_optical_frame'
                message.header.stamp.sec = 7
                fields = from_ros(message)
                self.assertIsInstance(fields['data'], bytes)
                self.assertEqual(as_bytes(fields['data']), pixels)
                self.assertEqual(fields['is_bigendian'], big_endian)
                self.assertEqual(fields['step'], 1280)
                self.assertEqual(fields['header']['frame_id'], 'camera_optical_frame')
                self.assertEqual(fields['header']['stamp']['sec'], 7)

    def test_compressed_camera_keeps_encoded_bytes_and_metadata(self):
        jpeg = b'\xff\xd8' + bytes(range(256)) * 100 + b'\xff\xd9'
        message = CompressedImage(format='jpeg', data=jpeg)
        fields = from_ros(message)
        self.assertEqual(fields['format'], 'jpeg')
        self.assertEqual(as_bytes(fields['data']), jpeg)
        self.assertIsInstance(fields['data'], bytes)

    def test_output_binary_arrays_preserve_all_bytes(self):
        pixels = bytes(range(256)) * 2700
        for message_type in (Image, CompressedImage, PointCloud2):
            with self.subTest(message_type=message_type):
                message = to_ros(message_type, {'data': base64.b64encode(pixels).decode()})
                self.assertEqual(bytes(message.data), pixels)

    def test_other_message_arrays_and_string_data_keep_their_types(self):
        camera = CameraInfo(k=[1., 0., 2., 0., 3., 4., 0., 0., 1.])
        self.assertEqual(from_ros(camera)['k'], list(camera.k))
        self.assertEqual(to_ros(String, {'data': 'not base64'}).data, 'not base64')


if __name__ == '__main__':
    unittest.main()
