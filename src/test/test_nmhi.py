import unittest
import numpy as np
import timeit

import stretch_pyfunmap.numba_height_image


class TestNumbaMHI(unittest.TestCase):

    def test_uint8mhi_to_points(self):
        """TODO
        """
        w, h = 100, 100
        T = np.eye(4)
        points = np.zeros((h * w,), dtype=[
                              ('x', np.float32),
                              ('y', np.float32),
                              ('z', np.float32)])
        m_per_pix = 0.01
        m_per_height_unit = 0.01

        # empty mhi
        image = np.zeros((h, w), dtype=np.uint8)
        num_points = stretch_pyfunmap.numba_height_image.numba_max_height_image_to_points(
            T, image, points, m_per_pix, m_per_height_unit)
        self.assertEqual(0, num_points)
        self.assertFalse((points['z'] == 1.0).all())

        # valid mhi
        image = np.full((h, w), 101, dtype=np.uint8)
        num_points = stretch_pyfunmap.numba_height_image.numba_max_height_image_to_points(
            T, image, points, m_per_pix, m_per_height_unit)
        self.assertEqual(h * w, num_points)
        self.assertTrue((points['z'] == 1.0).all())

        # random mhi
        image = np.random.randint(1, 255, size=(h, w), dtype=np.uint8)
        num_points = stretch_pyfunmap.numba_height_image.numba_max_height_image_to_points(
            T, image, points, m_per_pix, m_per_height_unit)
        self.assertEqual(h * w, num_points)
        self.assertTrue((points['z'] >= 0.0).all())
        self.assertTrue((points['z'] < 2.55).all())

    def test_performance_uint8mhi_to_points(self):
        """TODO
        """
        T = np.eye(4)
        m_per_pix = 0.01
        m_per_height_unit = 0.01

        # typical stress test
        w, h = 2000, 2000
        points = np.zeros((h * w,), dtype=[
                              ('x', np.float32),
                              ('y', np.float32),
                              ('z', np.float32)])

        image = np.random.randint(1, 255, size=(h, w), dtype=np.uint8)
        num_points = stretch_pyfunmap.numba_height_image.numba_max_height_image_to_points(
            T, image, points, m_per_pix, m_per_height_unit)
        self.assertEqual(h * w, num_points)
        self.assertTrue((points['z'] >= 0.0).all())
        self.assertTrue((points['z'] < 2.55).all())

        # absurd stress test
        w, h = 10000, 10000
        points = np.zeros((h * w,), dtype=[
                              ('x', np.float32),
                              ('y', np.float32),
                              ('z', np.float32)])

        image = np.random.randint(1, 255, size=(h, w), dtype=np.uint8)
        num_points = stretch_pyfunmap.numba_height_image.numba_max_height_image_to_points(
            T, image, points, m_per_pix, m_per_height_unit)
        self.assertEqual(h * w, num_points)
        self.assertTrue((points['z'] >= 0.0).all())
        self.assertTrue((points['z'] < 2.55).all())