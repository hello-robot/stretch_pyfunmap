import unittest
import numpy as np

import stretch_pyfunmap.max_height_image


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
        voi = stretch_pyfunmap.max_height_image.VolumeOfInterest('map', origin, axes, xy_m, xy_m, z_m)

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
        voi = stretch_pyfunmap.max_height_image.VolumeOfInterest('map', origin, axes, xy_m, xy_m, z_m)

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
        voi = stretch_pyfunmap.max_height_image.VolumeOfInterest('map', origin, axes, xy_m, xy_m, z_m)

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
        voi = stretch_pyfunmap.max_height_image.VolumeOfInterest('map', origin, axes, xy_m, xy_m, z_m)

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

    def test_change_frame_identity(self):
        """Create a VolumeOfInterest object and verify that the
        new attributes from a identity frame change are equal to
        the old attributes.
        """
        xy_m = 8.0
        z_m = 2.0
        voi_side_m = xy_m
        origin = np.array([-voi_side_m/2.0, -voi_side_m/2.0, -0.05])
        axes = np.eye(3)
        voi = stretch_pyfunmap.max_height_image.VolumeOfInterest('map', origin, axes, xy_m, xy_m, z_m)

        old_to_new_frame_mat = np.eye(4)
        new_frame = "new_map"
        expected_voi_to_new_map = np.copy(voi.points_in_voi_to_frame_id_mat)
        expected_new_map_to_voi = np.copy(voi.points_in_frame_id_to_voi_mat)
        voi.change_frame(old_to_new_frame_mat, new_frame)
        self.assertEqual("new_map", voi.frame_id)
        np.testing.assert_array_equal(origin, voi.origin)
        np.testing.assert_array_equal(axes, voi.axes)
        np.testing.assert_array_equal(expected_voi_to_new_map, voi.points_in_voi_to_frame_id_mat)
        np.testing.assert_array_equal(expected_new_map_to_voi, voi.points_in_frame_id_to_voi_mat)

    def test_change_frame_rotation(self):
        """Create a VolumeOfInterest object and verify that the
        new attributes from a 90 degree rotation along z axis
        frame change are correct.
        """
        xy_m = 8.0
        z_m = 2.0
        voi_side_m = xy_m
        origin = np.array([-voi_side_m/2.0, -voi_side_m/2.0, -0.05])
        axes = np.eye(3)
        voi = stretch_pyfunmap.max_height_image.VolumeOfInterest('map', origin, axes, xy_m, xy_m, z_m)

        rot = np.pi/2
        old_to_new_frame_mat = np.array([
            [np.cos(rot), -np.sin(rot), 0.0, 0.0],
            [np.sin(rot),  np.cos(rot), 0.0, 0.0],
            [0.0,                  0.0, 1.0, 0.0],
            [0.0,                  0.0, 0.0, 1.0]
        ])
        new_frame = "new_map"
        expected_origin = np.array([4.0, -4.0, -0.05])
        expected_axes = np.array([
            [0.0, -1.0, 0.0],
            [1.0,  0.0, 0.0],
            [0.0,  0.0, 1.0]
        ])
        expected_voi_to_new_map = np.array([
            [0.0, -1.0, 0.0,  4.0 ],
            [1.0,  0.0, 0.0, -4.0 ],
            [0.0,  0.0, 1.0, -0.05],
            [0.0,  0.0, 0.0,  1.0 ]
        ])
        expected_new_map_to_voi = np.array([
            [ 0.0, 1.0, 0.0, 4.0 ],
            [-1.0, 0.0, 0.0, 4.0 ],
            [ 0.0, 0.0, 1.0, 0.05],
            [ 0.0, 0.0, 0.0, 1.0 ]
        ])
        voi.change_frame(old_to_new_frame_mat, new_frame)
        self.assertEqual("new_map", voi.frame_id)
        np.testing.assert_allclose(expected_origin, voi.origin, atol=1e-7)
        np.testing.assert_allclose(expected_axes, voi.axes, atol=1e-7)
        np.testing.assert_allclose(expected_voi_to_new_map, voi.points_in_voi_to_frame_id_mat, atol=1e-7)
        np.testing.assert_allclose(expected_new_map_to_voi, voi.points_in_frame_id_to_voi_mat, atol=1e-7)

    def test_serialization(self):
        """Create a VolumeOfInterest object and verify
        the serialized object is correct.
        """
        xy_m = 8.0
        z_m = 2.0
        voi_side_m = xy_m
        origin = np.array([-voi_side_m/2.0, -voi_side_m/2.0, -0.05])
        axes = np.eye(3)
        voi = stretch_pyfunmap.max_height_image.VolumeOfInterest('map', origin, axes, xy_m, xy_m, z_m)

        expected_serialization = {
            'frame_id': 'map',
            'origin': list(origin.ravel()), # convert numpy array to List for assertDictEqual
            'axes': list(axes.ravel()), # convert numpy array to List for assertDictEqual
            'x_in_m': xy_m,
            'y_in_m': xy_m,
            'z_in_m': z_m
        }
        serialized = voi.serialize()
        serialized['origin'] = list(serialized['origin'].ravel())
        serialized['axes'] = list(serialized['axes'].ravel())
        self.assertDictEqual(expected_serialization, serialized)

    def test_serialization_roundtrip(self):
        """Test the `serialize()` and `from_serialization()` methods.
        """
        xy_m = 8.0
        z_m = 2.0
        voi_side_m = xy_m
        origin = np.array([-voi_side_m/2.0, -voi_side_m/2.0, -0.05])
        axes = np.eye(3)
        voi = stretch_pyfunmap.max_height_image.VolumeOfInterest('map', origin, axes, xy_m, xy_m, z_m)

        expected_serialization = {
            'frame_id': 'map',
            'origin': list(origin.ravel()), # convert numpy array to List for assertDictEqual
            'axes': list(axes.ravel()), # convert numpy array to List for assertDictEqual
            'x_in_m': xy_m,
            'y_in_m': xy_m,
            'z_in_m': z_m
        }
        new_voi = stretch_pyfunmap.max_height_image.VolumeOfInterest.from_serialization(voi.serialize())
        serialized = new_voi.serialize()
        serialized['origin'] = list(serialized['origin'].ravel())
        serialized['axes'] = list(serialized['axes'].ravel())
        self.assertDictEqual(expected_serialization, serialized)
