import unittest
import numpy as np
import math
import io
import contextlib

import stretch_funmap.max_height_image


class TestMHI(unittest.TestCase):

    def test_mhi_xy_bins(self):
        """Create a MaxHeightImage and verify the num x and y bins
        are correct.
        """
        # default voi
        xy_m = 8.0
        z_m = 2.0
        voi_side_m = xy_m
        origin = np.array([-voi_side_m/2.0, -voi_side_m/2.0, 0.0])
        axes = np.eye(3)
        voi = stretch_funmap.max_height_image.VolumeOfInterest('map', origin, axes, xy_m, xy_m, z_m)

        m_per_pix = 0.006
        pixel_dtype = np.uint8
        mhi = stretch_funmap.max_height_image.MaxHeightImage(voi, m_per_pix, pixel_dtype)
        num_x_bins = mhi.image.shape[1]
        num_y_bins = mhi.image.shape[0]
        expected_num_xy_bins = math.ceil(xy_m / m_per_pix) # ceil makes VOI fit inside MHI
        self.assertEqual(expected_num_xy_bins, num_x_bins)
        self.assertEqual(expected_num_xy_bins, num_y_bins)

        # rectangle voi
        voi = stretch_funmap.max_height_image.VolumeOfInterest('map', np.array([0, 0, 0]), np.eye(3), 4.0, 2.0, 1.0)
        m_per_pix = 0.1
        pixel_dtype = np.uint8
        mhi = stretch_funmap.max_height_image.MaxHeightImage(voi, m_per_pix, pixel_dtype)
        num_x_bins = mhi.image.shape[1]
        num_y_bins = mhi.image.shape[0]
        expected_num_x_bins = 40
        expected_num_y_bins = 20
        self.assertEqual(expected_num_x_bins, num_x_bins)
        self.assertEqual(expected_num_y_bins, num_y_bins)

    def test_mhi_data_types(self):
        """Create a MaxHeightImage of each supported integer data type
        and verify the autogenerated "meters per height unit" is correct.
        """
        xy_m = 8.0
        z_m = 2.0
        voi_side_m = xy_m
        origin = np.array([-voi_side_m/2.0, -voi_side_m/2.0, 0.0])
        axes = np.eye(3)
        voi = stretch_funmap.max_height_image.VolumeOfInterest('map', origin, axes, xy_m, xy_m, z_m)

        # uint8
        mhi = stretch_funmap.max_height_image.MaxHeightImage(voi, 0.006, np.uint8)
        expected_mphu = z_m / (256 - 2)
        self.assertEqual(expected_mphu, mhi.m_per_height_unit)

        # uint16
        mhi = stretch_funmap.max_height_image.MaxHeightImage(voi, 0.006, np.uint16)
        expected_mphu = z_m / (65535 - 1)
        self.assertEqual(expected_mphu, mhi.m_per_height_unit)

        # uint32
        mhi = stretch_funmap.max_height_image.MaxHeightImage(voi, 0.006, np.uint32)
        expected_mphu = z_m / (4294967295 - 1)
        self.assertEqual(expected_mphu, mhi.m_per_height_unit)

        # # uint64 - TODO prints warning
        # mhi = stretch_funmap.max_height_image.MaxHeightImage(voi, 0.006, np.uint64)
        # expected_mphu = z_m / (18446744073709551615 - 1)
        # self.assertEqual(expected_mphu, mhi.m_per_height_unit)

        # float32
        mhi = stretch_funmap.max_height_image.MaxHeightImage(voi, 0.006, np.float32)
        expected_mphu = None
        self.assertEqual(expected_mphu, mhi.m_per_height_unit)

        # float64
        mhi = stretch_funmap.max_height_image.MaxHeightImage(voi, 0.006, np.float64)
        expected_mphu = None
        self.assertEqual(expected_mphu, mhi.m_per_height_unit)

    def test_mhi_invalid_data_types(self):
        """Create a MaxHeightImage with invalid data types.
        """
        xy_m = 8.0
        z_m = 2.0
        voi_side_m = xy_m
        origin = np.array([-voi_side_m/2.0, -voi_side_m/2.0, 0.0])
        axes = np.eye(3)
        voi = stretch_funmap.max_height_image.VolumeOfInterest('map', origin, axes, xy_m, xy_m, z_m)

        # builtin int
        with self.assertRaises(AssertionError), contextlib.redirect_stdout(io.StringIO()):
            mhi = stretch_funmap.max_height_image.MaxHeightImage(voi, 0.006, int)

        # builtin float
        with self.assertRaises(AssertionError), contextlib.redirect_stdout(io.StringIO()):
            mhi = stretch_funmap.max_height_image.MaxHeightImage(voi, 0.006, float)

    def test_mhi_data_types_w_invalid_mphu(self):
        """Create a MaxHeightImage with invalid data type / "meters
        per height unit" combos.
        """
        xy_m = 8.0
        z_m = 2.0
        voi_side_m = xy_m
        origin = np.array([-voi_side_m/2.0, -voi_side_m/2.0, 0.0])
        axes = np.eye(3)
        voi = stretch_funmap.max_height_image.VolumeOfInterest('map', origin, axes, xy_m, xy_m, z_m)

        # valid mphu
        f = io.StringIO()
        with contextlib.redirect_stdout(f):
            expected_mphu = z_m / (256 - 2)
            mhi = stretch_funmap.max_height_image.MaxHeightImage(voi, 0.006, np.uint8, m_per_height_unit=expected_mphu)
        self.assertEqual(0, len(f.getvalue()))

        # invalid mphu
        f = io.StringIO()
        with contextlib.redirect_stdout(f):
            expected_mphu = z_m / (256 - 1)
            mhi = stretch_funmap.max_height_image.MaxHeightImage(voi, 0.006, np.uint8, m_per_height_unit=expected_mphu)
        self.assertNotEqual(0, len(f.getvalue()))

    def test_mhi_images(self):
        """Create a MaxHeightImage and verify the images are correct.
        """
        xy_m = 8.0
        z_m = 2.0
        voi_side_m = xy_m
        origin = np.array([-voi_side_m/2.0, -voi_side_m/2.0, 0.0])
        axes = np.eye(3)
        voi = stretch_funmap.max_height_image.VolumeOfInterest('map', origin, axes, xy_m, xy_m, z_m)

        mhi = stretch_funmap.max_height_image.MaxHeightImage(voi, 0.006, np.uint8, use_camera_depth_image=True)
        self.assertTrue((mhi.image == 0).all())
        self.assertTrue((mhi.camera_depth_image == 0).all())

    def test_mhi_image_origin(self):
        """Create a MaxHeightImage and verify the image origin is correct.
        """
        xy_m = 8.0
        z_m = 2.0
        voi_side_m = xy_m
        origin = np.array([-voi_side_m/2.0, -voi_side_m/2.0, 0.0])
        axes = np.eye(3)
        voi = stretch_funmap.max_height_image.VolumeOfInterest('map', origin, axes, xy_m, xy_m, z_m)

        m_per_pix = 0.006
        mhi = stretch_funmap.max_height_image.MaxHeightImage(voi, m_per_pix, np.uint8, use_camera_depth_image=True)
        expected_image_origin = np.array([0.012, 8.0-0.012, 0.0])
        np.testing.assert_array_equal(expected_image_origin, mhi.image_origin)