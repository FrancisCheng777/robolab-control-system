import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vendor"))
from legacy_cameras import KinectV2, RealSenseRGB


class CameraTimestampTests(unittest.TestCase):
    def test_realsense_keeps_device_domain_and_host_receipt(self):
        camera = RealSenseRGB.__new__(RealSenseRGB)
        frame = Mock()
        frame.get_data.return_value = np.array([[[3, 2, 1]]], np.uint8)
        frame.get_timestamp.return_value = 1234500.
        frame.get_frame_number.return_value = 42
        frame.get_frame_timestamp_domain.return_value = "hardware_clock"
        camera.pipe = Mock()
        camera.pipe.wait_for_frames.return_value.get_color_frame.return_value = frame
        with patch("legacy_cameras.time.monotonic", return_value=10.):
            image, meta = camera.read_with_metadata()
        np.testing.assert_array_equal(image, [[[1, 2, 3]]])
        self.assertEqual(meta["source_time_s"], 1234.5)
        self.assertEqual(meta["receive_monotonic_s"], 10.)
        self.assertEqual(meta["source_clock"], "realsense_hardware_clock")
        self.assertEqual(meta["frame_id"], 42)

    def test_kinect_125us_units_and_frame_release(self):
        camera = KinectV2.__new__(KinectV2)
        camera.rotate_cw = camera.crop_square = False
        frame = Mock(timestamp=8000, sequence=42)
        frame.asarray.return_value = np.array([[[3, 2, 1, 0]]], np.uint8)
        camera.listener = Mock()
        frames = {"color": frame}
        camera.listener.waitForNewFrame.return_value = frames
        with patch("legacy_cameras.time.monotonic", return_value=10.):
            image, meta = camera.read_with_metadata()
        np.testing.assert_array_equal(image, [[[1, 2, 3]]])
        self.assertEqual(meta["source_time_s"], 1.)
        self.assertEqual(meta["receive_monotonic_s"], 10.)
        camera.listener.release.assert_called_once_with(frames)

    def test_kinect_rotation_crop_rgb_and_sdk_buffer_ownership(self):
        cases = [
            (2, 4, False, False, [[1, 2, 3, 4], [5, 6, 7, 8]]),
            (2, 4, True, False, [[5, 1], [6, 2], [7, 3], [8, 4]]),
            (2, 4, False, True, [[2, 3], [6, 7]]),
            (2, 4, True, True, [[6, 2], [7, 3]]),
            (3, 4, True, True, [[9, 5, 1], [10, 6, 2], [11, 7, 3]]),
        ]
        for height, width, rotate, crop, expected_red in cases:
            with self.subTest(height=height, width=width, rotate=rotate, crop=crop):
                red = np.arange(1, height * width + 1, dtype=np.uint8).reshape(height, width)
                buffer = np.stack((red + 100, red + 50, red, np.full_like(red, 255)), axis=-1)
                frame = Mock(timestamp=8000, sequence=42)
                frame.asarray.return_value = buffer
                camera = KinectV2.__new__(KinectV2)
                camera.rotate_cw, camera.crop_square = rotate, crop
                camera.listener = Mock()
                frames = {"color": frame}
                camera.listener.waitForNewFrame.return_value = frames
                # Simulate the SDK immediately reusing its buffer on release.
                camera.listener.release.side_effect = lambda _: buffer.fill(0)
                image, metadata = camera.read_with_metadata()
                expected = np.array(expected_red, dtype=np.uint8)
                np.testing.assert_array_equal(image[..., 0], expected)
                np.testing.assert_array_equal(image[..., 1], expected + 50)
                np.testing.assert_array_equal(image[..., 2], expected + 100)
                self.assertTrue(image.flags.c_contiguous, "avoid another full-image copy during resize")
                self.assertFalse(np.shares_memory(image, buffer))
                self.assertEqual(metadata["frame_id"], 42)
                camera.listener.release.assert_called_once_with(frames)

    def test_kinect_releases_frame_when_conversion_fails(self):
        camera = KinectV2.__new__(KinectV2)
        camera.rotate_cw = camera.crop_square = False
        frame = Mock(timestamp=8000, sequence=42)
        frame.asarray.side_effect = RuntimeError("bad frame")
        camera.listener = Mock()
        frames = {"color": frame}
        camera.listener.waitForNewFrame.return_value = frames
        with self.assertRaisesRegex(RuntimeError, "bad frame"):
            camera.read_with_metadata()
        camera.listener.release.assert_called_once_with(frames)

    def test_kinect_serial_uses_bytes_for_sdk(self):
        for serial in ("019337540647", b"019337540647", None):
            with self.subTest(serial=serial):
                fn = Mock()
                fn.enumerateDevices.return_value = 1
                fn.getDefaultDeviceSerialNumber.return_value = b"019337540647"
                sdk = SimpleNamespace(Freenect2=Mock(return_value=fn),
                    SyncMultiFrameListener=Mock(), Frame=Mock(), FrameType=Mock(),
                    CpuPacketPipeline=Mock(), Registration=Mock())
                with patch.dict(sys.modules, pylibfreenect2=sdk), patch("legacy_cameras._preload_freenect2"):
                    camera = KinectV2(serial=serial)
                self.assertEqual(camera.serial, "019337540647")
                self.assertEqual(fn.openDevice.call_args.args[0], b"019337540647")
                camera.close()
