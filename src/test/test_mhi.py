import unittest
import numpy as np
import math

import stretch_funmap.max_height_image


class TestMHI(unittest.TestCase):

    def test_mhi_xy_bins(self):
        """Create a MaxHeightImage and verify the num x and y bins
        are correct.
        """
        xy_m = 8.0
        z_m = 2.0
        voi_side_m = xy_m
        origin = np.array([-voi_side_m/2.0, -voi_side_m/2.0, -0.05])
        axes = np.eye(3)
        voi = stretch_funmap.max_height_image.VolumeOfInterest('map', origin, axes, xy_m, xy_m, z_m)

        m_per_pix = 0.006
        pixel_dtype = np.uint8
        mhi = stretch_funmap.max_height_image.MaxHeightImage(voi, m_per_pix, pixel_dtype, use_camera_depth_image=True)
        num_x_bins = mhi.image.shape[1]
        num_y_bins = mhi.image.shape[0]
        expected_num_xy_bins = math.ceil(xy_m / m_per_pix) # ceil makes VOI fit inside MHI
        self.assertEqual(expected_num_xy_bins, num_x_bins)
        self.assertEqual(expected_num_xy_bins, num_y_bins)

    @unittest.skip(reason="TODO")
    def test_mhi_data_types(self):
        """Create a MaxHeightImage of each supported integer data type
        and verify the "meters per type unit" is correct.
        """
        xy_m = 8.0
        z_m = 2.0
        voi_side_m = xy_m
        origin = np.array([-voi_side_m/2.0, -voi_side_m/2.0, -0.05])
        axes = np.eye(3)
        voi = stretch_funmap.max_height_image.VolumeOfInterest('map', origin, axes, xy_m, xy_m, z_m)

        m_per_pix = 0.006
        pixel_dtype = np.uint8
        mhi = stretch_funmap.max_height_image.MaxHeightImage(voi, m_per_pix, pixel_dtype, use_camera_depth_image=True)
        self.assertTrue(False)

    @unittest.skip(reason="TODO")
    def test_mhi_invalid_data_types(self):
        """Create a MaxHeightImage with invalid data types.
        """
        self.assertTrue(False)

    @unittest.skip(reason="TODO")
    def test_mhi_data_types_w_invalid_mptu(self):
        """Create a MaxHeightImage with invalid data type / "meters
        per type unit" combos.
        """
        self.assertTrue(False)

    @unittest.skip(reason="TODO")
    def test_mhi_images(self):
        """Create a MaxHeightImage and verify the images are correct.
        """
        self.assertTrue(False)

    @unittest.skip(reason="TODO")
    def test_mhi_image_origin(self):
        """Create a MaxHeightImage and verify the image origin is correct.
        """
        self.assertTrue(False)
