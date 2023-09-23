import pickle
import timeit
import unittest
import numpy as np
from pathlib import Path
from test.utils import ProgressTimer

import stretch_pyfunmap.segment_max_height_image as sm


class TestSegmentObjects(unittest.TestCase):

    def get_surface(self, manip):
        height, width = manip.max_height_im.image.shape
        robot_xy_pix = [width/2, 0]
        surface_mask, plane_parameters = sm.find_closest_flat_surface(manip.max_height_im, robot_xy_pix)
        return surface_mask, plane_parameters

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

    def get_object(self, surface, plane_parameters, manip):
        grasp_target = sm.find_object_to_grasp(surface, plane_parameters, manip.max_height_im)
        return grasp_target

    def test_find_grasp_target_basic(self):
        """Load a manipulation view with a known surface and known object
        and verify a grasp target is found.
        """
        pkl_fpath = Path(__file__).parent / 'assets' / 'segment_objects' / 'manip_found.pkl'
        with open(str(pkl_fpath), 'rb') as inp:
            manip_found = pickle.load(inp)

        surface_mask, plane_parameters = self.get_surface(manip_found)
        grasp_target = self.get_object(surface_mask, plane_parameters, manip_found)
        self.assertTrue(grasp_target is not None)
