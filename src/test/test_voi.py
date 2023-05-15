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

    def test_voi_frame_projection(self):
        """Create a VolumeOfInterest object and verify that projected
        "points wrt voi to frame" and "points wrt frame to voi"
        are correct.
        """
        xy_m = 8.0
        z_m = 2.0
        voi_side_m = xy_m
        origin = np.array([-voi_side_m/2.0, -voi_side_m/2.0, -0.05])
        axes = np.eye(3)
        voi = stretch_funmap.max_height_image.VolumeOfInterest('map', origin, axes, xy_m, xy_m, z_m)

        # test one point
        points_wrt_voi = np.array([
            [0.0],
            [0.0],
            [0.0]
        ])
        expected_points_wrt_map = np.array([
            [-4.0],
            [-4.0],
            [-0.05]
        ])
        points_wrt_map = (voi.points_in_voi_to_frame_id_mat @ np.vstack((points_wrt_voi, np.ones(points_wrt_voi.shape[1]))))[:3,:]
        points_wrt_voi_again = (voi.points_in_frame_id_to_voi_mat @ np.vstack((points_wrt_map, np.ones(points_wrt_map.shape[1]))))[:3,:]
        np.testing.assert_array_equal(expected_points_wrt_map, points_wrt_map)
        np.testing.assert_array_equal(points_wrt_voi, points_wrt_voi_again)

        # test two points
        points_wrt_voi = np.array([
            [0.0, 1.0],
            [0.0, 1.0],
            [0.0, 1.0]
        ])
        expected_points_wrt_map = np.array([
            [-4.0, -3.0],
            [-4.0, -3.0],
            [-0.05, 0.95]
        ])
        points_wrt_map = (voi.points_in_voi_to_frame_id_mat @ np.vstack((points_wrt_voi, np.ones(points_wrt_voi.shape[1]))))[:3,:]
        points_wrt_voi_again = (voi.points_in_frame_id_to_voi_mat @ np.vstack((points_wrt_map, np.ones(points_wrt_map.shape[1]))))[:3,:]
        np.testing.assert_array_equal(expected_points_wrt_map, points_wrt_map)
        np.testing.assert_array_equal(points_wrt_voi, points_wrt_voi_again)

        # test three points
        points_wrt_voi = np.array([
            [0.0, 0.0, 4.0],
            [0.0, 0.0, 4.0],
            [0.0, 0.0, 0.05]
        ])
        expected_points_wrt_map = np.array([
            [-4.0, -4.0, 0.0],
            [-4.0, -4.0, 0.0],
            [-0.05, -0.05, 0.0]
        ])
        points_wrt_map = (voi.points_in_voi_to_frame_id_mat @ np.vstack((points_wrt_voi, np.ones(points_wrt_voi.shape[1]))))[:3,:]
        points_wrt_voi_again = (voi.points_in_frame_id_to_voi_mat @ np.vstack((points_wrt_map, np.ones(points_wrt_map.shape[1]))))[:3,:]
        np.testing.assert_array_equal(expected_points_wrt_map, points_wrt_map)
        np.testing.assert_array_equal(points_wrt_voi, points_wrt_voi_again)

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
        camera_to_map_mat = np.array([
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
        np.testing.assert_array_equal(expected_points_to_voi_mat, voi.get_points_to_voi_matrix(camera_to_map_mat))

    def test_points_voi_projection(self):
        """Create a VolumeOfInterest object and verify that points
        projected with the "points to voi" transformation matrix from `get_points_to_voi_matrix()`
        is correct.
        """
        xy_m = 8.0
        z_m = 2.0
        voi_side_m = xy_m
        origin = np.array([-voi_side_m/2.0, -voi_side_m/2.0, -0.05])
        axes = np.eye(3)
        voi = stretch_funmap.max_height_image.VolumeOfInterest('map', origin, axes, xy_m, xy_m, z_m)

        # test one point - camera frame shifted +1m on z axis
        shift = 1.0
        camera_to_map_mat = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, shift],
            [0.0, 0.0, 0.0, 1.0]
        ])
        camera_to_voi_mat = voi.get_points_to_voi_matrix(camera_to_map_mat)
        points_wrt_camera = np.array([
            [0.0],
            [0.0],
            [0.0]
        ])
        points_wrt_voi = (camera_to_voi_mat @ np.vstack((points_wrt_camera, np.ones(points_wrt_camera.shape[1]))))[:3,:]
        expected_points_wrt_voi = np.array([
            [4.0],
            [4.0],
            [1.05]
        ])
        np.testing.assert_array_equal(expected_points_wrt_voi, points_wrt_voi)

        # test two points - camera frame rotated 90 degrees on z axis
        rot = np.pi/2
        camera_to_map_mat = np.array([
            [np.cos(rot), -np.sin(rot), 0.0, 0.0],
            [np.sin(rot),  np.cos(rot), 0.0, 0.0],
            [0.0,                  0.0, 1.0, 0.0],
            [0.0,                  0.0, 0.0, 1.0]
        ])
        camera_to_voi_mat = voi.get_points_to_voi_matrix(camera_to_map_mat)
        points_wrt_camera = np.array([
            [0.0, 1.0],
            [0.0, 1.0],
            [0.0, 1.0]
        ])
        points_wrt_voi = (camera_to_voi_mat @ np.vstack((points_wrt_camera, np.ones(points_wrt_camera.shape[1]))))[:3,:]
        expected_points_wrt_voi = np.array([
            [4.0, 3.0],
            [4.0, 5.0],
            [0.05, 1.05]
        ])
        np.testing.assert_array_equal(expected_points_wrt_voi, points_wrt_voi)

        # test three points - camera frame rotated 90 degrees on z axis, shifted 1m along x and z axes
        rot = np.pi/2
        shift = 1.0
        camera_to_map_mat = np.array([
            [np.cos(rot), -np.sin(rot), 0.0, shift],
            [np.sin(rot),  np.cos(rot), 0.0, 0.0],
            [0.0,                  0.0, 1.0, shift],
            [0.0,                  0.0, 0.0, 1.0]
        ])
        camera_to_voi_mat = voi.get_points_to_voi_matrix(camera_to_map_mat)
        points_wrt_camera = np.array([
            [0.0, 1.0, -4],
            [0.0, 1.0, 5],
            [0.0, 1.0, -1.05]
        ])
        points_wrt_voi = (camera_to_voi_mat @ np.vstack((points_wrt_camera, np.ones(points_wrt_camera.shape[1]))))[:3,:]
        expected_points_wrt_voi = np.array([
            [5.0, 4.0, 0.0],
            [4.0, 5.0, 0.0],
            [1.05, 2.05, 0.0]
        ])
        np.testing.assert_allclose(expected_points_wrt_voi, points_wrt_voi, atol=1e-7)
