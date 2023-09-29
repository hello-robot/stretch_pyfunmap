import pickle
import timeit
import unittest
import numpy as np
from pathlib import Path
from test.utils import ProgressTimer

import stretch_pyfunmap.segment_max_height_image as sm


class TestSegmentObjects(unittest.TestCase):

    def setUp(self):
        """Setup the analysis of past performance to help detect outliers.

        Contribute your results here.
        LOG:
         - Sept 17th, 2023 on a Stretch RE1 with ./assets/segment_objects/manip_found.pkl
        """
        # TODO: collect enough data and log here
        # self.get_surface_exectime_ms = np.array([31.77, 29.89])
        # self.get_surface_exectime_ms_mean = np.mean(self.get_surface_exectime_ms)
        # self.get_surface_exectime_ms_std = np.std(self.get_surface_exectime_ms)
        # self.get_object_exectime_ms = np.array([60.48, 47.92])
        # self.get_object_exectime_ms_mean = np.mean(self.get_object_exectime_ms)
        # self.get_object_exectime_ms_std = np.std(self.get_object_exectime_ms)
        # self.get_objects_exectime_ms = np.array([60.48, 47.92])
        # self.get_objects_exectime_ms_mean = np.mean(self.get_objects_exectime_ms)
        # self.get_objects_exectime_ms_std = np.std(self.get_objects_exectime_ms)

    def get_surface(self, manip):
        height, width = manip.max_height_im.image.shape
        robot_xy_pix = [width/2, 0]
        surface_mask, plane_parameters = sm.find_closest_flat_surface(manip.max_height_im, robot_xy_pix)
        return surface_mask, plane_parameters

    def timeit_get_surface(self, manip, n=1e4):
        n = int(n)
        total = ProgressTimer(lambda: self.get_surface(manip)).timeit(number=n)
        single = total / n
        single_ms = single * 1000
        return single_ms

    def test_find_surface_performance1(self):
        """Use timeit to measure the # of ms it takes to execute this
        method with observations from manip_found.pkl.
        Expected runtime is ~5 minutes.
        """
        pkl_fpath = Path(__file__).parent / 'assets' / 'segment_objects' / 'manip_found.pkl'
        with open(str(pkl_fpath), 'rb') as inp:
            manip_found = pickle.load(inp)

        single_ms = self.timeit_get_surface(manip_found)
        print(f"Get Surface: {single_ms:.2f} ms (Add your result to get_surface_exectime_ms in test_segment_objects_perf.setUp() for ./assets/segment_objects/manip_found.pkl)")
        # TODO: collect enough data to identify outliers
        # self.assertTrue(single_ms > self.get_surface_exectime_ms_mean - 3.0 * self.get_surface_exectime_ms_std)
        # self.assertTrue(single_ms < self.get_surface_exectime_ms_mean + 3.0 * self.get_surface_exectime_ms_std)

    def test_find_surface_performance2(self):
        """Use timeit to measure the # of ms it takes to execute this
        method with observations from manip_two.pkl.
        Expected runtime is ~5 minutes.
        """
        pkl_fpath = Path(__file__).parent / 'assets' / 'segment_objects' / 'manip_two.pkl'
        with open(str(pkl_fpath), 'rb') as inp:
            manip_two = pickle.load(inp)

        single_ms = self.timeit_get_surface(manip_two)
        print(f"Get Surface: {single_ms:.2f} ms (Add your result to get_surface_exectime_ms in test_segment_objects_perf.setUp() for ./assets/segment_objects/manip_two.pkl)")
        # TODO: collect enough data to identify outliers
        # self.assertTrue(single_ms > self.get_surface_exectime_ms_mean - 3.0 * self.get_surface_exectime_ms_std)
        # self.assertTrue(single_ms < self.get_surface_exectime_ms_mean + 3.0 * self.get_surface_exectime_ms_std)

    def get_object(self, surface, plane_parameters, manip):
        grasp_target = sm.find_object_to_grasp(surface, plane_parameters, manip.max_height_im)
        return grasp_target

    def timeit_get_object(self, surface, plane_parameters, manip, n=1e4):
        n = int(n)
        total = ProgressTimer(lambda: self.get_object(surface, plane_parameters, manip)).timeit(number=n)
        single = total / n
        single_ms = single * 1000
        return single_ms

    def test_find_grasp_target_performance1(self):
        """Use timeit to measure the # of ms it takes to execute this
        method with observations from manip_found.pkl.
        Expected runtime is ~1 minute.
        """
        pkl_fpath = Path(__file__).parent / 'assets' / 'segment_objects' / 'manip_found.pkl'
        with open(str(pkl_fpath), 'rb') as inp:
            manip_found = pickle.load(inp)

        surface_mask, plane_parameters = self.get_surface(manip_found)
        single_ms = self.timeit_get_object(surface_mask, plane_parameters, manip_found, n=1e2)
        print(f"Get Object: {single_ms:.2f} ms (Add your result to get_object_exectime_ms in test_segment_objects_perf.setUp() for ./assets/segment_objects/manip_found.pkl)")
        # TODO: collect enough data to identify outliers
        # self.assertTrue(single_ms > self.get_object_exectime_ms_mean - 3.0 * self.get_object_exectime_ms_std)
        # self.assertTrue(single_ms < self.get_object_exectime_ms_mean + 3.0 * self.get_object_exectime_ms_std)

    def test_find_grasp_target_performance2(self):
        """Use timeit to measure the # of ms it takes to execute this
        method with observations from manip_two.pkl.
        Expected runtime is ~10 minutes.
        """
        pkl_fpath = Path(__file__).parent / 'assets' / 'segment_objects' / 'manip_two.pkl'
        with open(str(pkl_fpath), 'rb') as inp:
            manip_two = pickle.load(inp)

        surface_mask, plane_parameters = self.get_surface(manip_two)
        single_ms = self.timeit_get_object(surface_mask, plane_parameters, manip_two)
        print(f"Get Object: {single_ms:.2f} ms (Add your result to get_object_exectime_ms in test_segment_objects_perf.setUp() for ./assets/segment_objects/manip_two.pkl)")
        # TODO: collect enough data to identify outliers
        # self.assertTrue(single_ms > self.get_object_exectime_ms_mean - 3.0 * self.get_object_exectime_ms_std)
        # self.assertTrue(single_ms < self.get_object_exectime_ms_mean + 3.0 * self.get_object_exectime_ms_std)

    def get_objects(self, surface, plane_parameters, manip):
        grasp_targets = sm.find_objects_to_grasp(surface, plane_parameters, manip.max_height_im)
        return grasp_targets

    def timeit_get_objects(self, surface, plane_parameters, manip, n=1e4):
        n = int(n)
        total = ProgressTimer(lambda: self.get_objects(surface, plane_parameters, manip)).timeit(number=n)
        single = total / n
        single_ms = single * 1000
        return single_ms

    def test_find_grasp_targets_performance1(self):
        """Use timeit to measure the # of ms it takes to execute this
        method with observations from manip_found.pkl.
        Expected runtime is ~1 minute.
        """
        pkl_fpath = Path(__file__).parent / 'assets' / 'segment_objects' / 'manip_found.pkl'
        with open(str(pkl_fpath), 'rb') as inp:
            manip_found = pickle.load(inp)

        surface_mask, plane_parameters = self.get_surface(manip_found)
        single_ms = self.timeit_get_objects(surface_mask, plane_parameters, manip_found, n=1e2)
        print(f"Get Objects: {single_ms:.2f} ms (Add your result to get_objects_exectime_ms in test_segment_objects_perf.setUp() for ./assets/segment_objects/manip_found.pkl)")
        # TODO: collect enough data to identify outliers
        # self.assertTrue(single_ms > self.get_objects_exectime_ms_mean - 3.0 * self.get_objects_exectime_ms_std)
        # self.assertTrue(single_ms < self.get_objects_exectime_ms_mean + 3.0 * self.get_objects_exectime_ms_std)

    def test_find_grasp_targets_performance2(self):
        """Use timeit to measure the # of ms it takes to execute this
        method with observations from manip_two.pkl.
        Expected runtime is ~10 minutes.
        """
        pkl_fpath = Path(__file__).parent / 'assets' / 'segment_objects' / 'manip_two.pkl'
        with open(str(pkl_fpath), 'rb') as inp:
            manip_two = pickle.load(inp)

        surface_mask, plane_parameters = self.get_surface(manip_two)
        single_ms = self.timeit_get_objects(surface_mask, plane_parameters, manip_two)
        print(f"Get Objects: {single_ms:.2f} ms (Add your result to get_objects_exectime_ms in test_segment_objects_perf.setUp() for ./assets/segment_objects/manip_two.pkl)")
        # TODO: collect enough data to identify outliers
        # self.assertTrue(single_ms > self.get_objects_exectime_ms_mean - 3.0 * self.get_objects_exectime_ms_std)
        # self.assertTrue(single_ms < self.get_objects_exectime_ms_mean + 3.0 * self.get_objects_exectime_ms_std)