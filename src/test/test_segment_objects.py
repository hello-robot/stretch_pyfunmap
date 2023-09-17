import pickle
import timeit
import unittest
import numpy as np
from pathlib import Path

import stretch_pyfunmap.segment_max_height_image as sm


class TestSegmentObjects(unittest.TestCase):

    def test_find_surface_basic(self):
        """Load a manipulation view with a known surface and verify
        a surface is found.
        """
        pkl_fpath = Path(__file__).parent / 'assets' / 'segment_objects' / 'manip_found.pkl'
        with open(str(pkl_fpath), 'rb') as inp:
            manip_found = pickle.load(inp)

        def get_surface(manip):
            height, width = manip.max_height_im.image.shape
            robot_xy_pix = [width/2, 0]
            surface_mask, plane_parameters = sm.find_closest_flat_surface(manip.max_height_im, robot_xy_pix)
            return surface_mask, plane_parameters

        surface_mask, plane_parameters = get_surface(manip_found)
        self.assertTrue(surface_mask is not None)
        self.assertTrue(plane_parameters is not None)

