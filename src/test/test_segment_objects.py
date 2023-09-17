import pickle
import timeit
import unittest
import numpy as np
from pathlib import Path

import stretch_pyfunmap.segment_max_height_image as sm


class TestSegmentObjects(unittest.TestCase):

    def setUp(self):
        """Setup the analysis of past performance to help detect outliers.

        Contribute your results here.
        LOG:
         - Sept 17th, 2023 on a Stretch RE1 with ./assets/segment_objects/manip_found.pkl
        """
        # TODO:
        # self.get_surface_exectime_ms = np.array([31.77, 29.89])
        # self.get_surface_exectime_ms_mean = np.mean(self.get_surface_exectime_ms)
        # self.get_surface_exectime_ms_std = np.std(self.get_surface_exectime_ms)
        # self.get_object_exectime_ms = np.array([])
        # self.get_object_exectime_ms_mean = np.mean(self.get_object_exectime_ms)
        # self.get_object_exectime_ms_std = np.std(self.get_object_exectime_ms)

    def get_surface(self, manip):
        height, width = manip.max_height_im.image.shape
        robot_xy_pix = [width/2, 0]
        surface_mask, plane_parameters = sm.find_closest_flat_surface(manip.max_height_im, robot_xy_pix)
        return surface_mask, plane_parameters

    def timeit_get_surface(self, manip, n=1e4):
        n = int(n)
        total = timeit.timeit(lambda: self.get_surface(manip), number=n)
        single = total / n
        single_ms = single * 1000
        return single_ms

    def test_find_surface_basic(self):
        """Load a manipulation view with a known surface and verify
        a surface is found.
        """
        pkl_fpath = Path(__file__).parent / 'assets' / 'segment_objects' / 'manip_found.pkl'
        with open(str(pkl_fpath), 'rb') as inp:
            manip_found = pickle.load(inp)

        surface_mask, plane_parameters = self.get_surface(manip_found)
        self.assertTrue(surface_mask is not None)
        self.assertTrue(plane_parameters is not None)

    def test_find_surface_performance1(self):
        """Use timeit to measure the # of ms it takes to execute this
        method with observations from manip_found.pkl.
        """
        pkl_fpath = Path(__file__).parent / 'assets' / 'segment_objects' / 'manip_found.pkl'
        with open(str(pkl_fpath), 'rb') as inp:
            manip_found = pickle.load(inp)

        single_ms = self.timeit_get_surface(manip_found)
        print(f"Get Surface: {single_ms:.2f} ms (Add your result to setUp() in test_segment_objects)")
        # TODO: identify outlier
        # self.assertTrue(single_ms > self.get_surface_exectime_ms_mean - 3.0 * self.get_surface_exectime_ms_std)
        # self.assertTrue(single_ms < self.get_surface_exectime_ms_mean + 3.0 * self.get_surface_exectime_ms_std)
