import unittest
import numpy as np

import stretch_funmap.max_height_image


class TestVOI(unittest.TestCase):

    def test_voi_frame_transformations(self):
        """Create a VolumeOfInterest object and verify that the
        "voi to frame" and "frame to voi" transformation matrices
        are correct. 
        """
        xy_m = 8.0
        z_m = 2.0
        voi_side_m = xy_m
        origin = np.array([-voi_side_m/2.0, -voi_side_m/2.0, -0.05])
        axes = np.eye(3)
        voi = stretch_funmap.max_height_image.VolumeOfInterest('map', origin, axes, xy_m, xy_m, z_m)

        expected_voi_to_map_mat = np.array([
            [1.0, 0.0, 0.0, -4.0],
            [0.0, 1.0, 0.0, -4.0],
            [0.0, 0.0, 1.0, -0.05],
            [0.0, 0.0, 0.0, 1.0]
        ])
        np.testing.assert_array_equal(expected_voi_to_map_mat, voi.points_in_voi_to_frame_id_mat)

        expected_map_to_voi_mat = np.array([
            [1.0, 0.0, 0.0, 4.0],
            [0.0, 1.0, 0.0, 4.0],
            [0.0, 0.0, 1.0, 0.05],
            [0.0, 0.0, 0.0, 1.0]
        ])
        np.testing.assert_array_equal(expected_map_to_voi_mat, voi.points_in_frame_id_to_voi_mat)

    def test_points_voi_transformations(self):
        """Create a VolumeOfInterest object and verify that the
        "points to voi" transformation matrix from `get_points_to_voi_matrix()`
        is correct.
        """
        xy_m = 8.0
        z_m = 2.0
        voi_side_m = xy_m
        origin = np.array([-voi_side_m/2.0, -voi_side_m/2.0, -0.05])
        axes = np.eye(3)
        voi = stretch_funmap.max_height_image.VolumeOfInterest('map', origin, axes, xy_m, xy_m, z_m)

        # give the identity and expect the same matrix
        expected_points_to_voi_mat = np.array([
            [1.0, 0.0, 0.0, 4.0],
            [0.0, 1.0, 0.0, 4.0],
            [0.0, 0.0, 1.0, 0.05],
            [0.0, 0.0, 0.0, 1.0]
        ])
        np.testing.assert_array_equal(expected_points_to_voi_mat, voi.get_points_to_voi_matrix(np.eye(4)))

        # give +1m on z axis, and expect the points to voi matrix to shift points up along z an extra 1m
        points_to_frame_mat = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 1.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        expected_points_to_voi_mat = np.array([
            [1.0, 0.0, 0.0, 4.0],
            [0.0, 1.0, 0.0, 4.0],
            [0.0, 0.0, 1.0, 1.05],
            [0.0, 0.0, 0.0, 1.0]
        ])
        np.testing.assert_array_equal(expected_points_to_voi_mat, voi.get_points_to_voi_matrix(points_to_frame_mat))
