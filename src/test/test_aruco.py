import unittest
import cv2
import yaml
from yaml.loader import SafeLoader
import pickle
from pathlib import Path

import stretch_pyfunmap.aruco


class TestAruco(unittest.TestCase):

    def test_invalid_aruco_id(self):
        """Try creating a invalid Aruco ID and confirm
        that a ValueError is thrown.
        """
        frame_id = 'camera_color_optical_frame'
        marker_info_yaml_fpath = Path(__file__).parent / 'assets' / 'aruco_detection' / 'aruco_marker_info.yaml'
        with open(str(marker_info_yaml_fpath)) as f:
            marker_info = yaml.load(f, Loader=SafeLoader)
        # 130 is a valid ID that exists in the marker info dictionary
        try:
            marker = stretch_pyfunmap.aruco.ArucoMarker(130, marker_info, frame_id)
        except ValueError:
            self.fail("shouldn't have raised a ValueError when given valid ID 130")

        # 0 is not a ID that exists in the marker info dictionary
        with self.assertRaises(ValueError):
            marker = stretch_pyfunmap.aruco.ArucoMarker(0, marker_info, frame_id)

    def test_temp(self):
        """TODO
        """
        # establish filepaths
        marker_info_yaml_fpath = Path(__file__).parent / 'assets' / 'aruco_detection' / 'aruco_marker_info.yaml'
        color_camera_info_pkl_fpath = Path(__file__).parent / 'assets' / 'aruco_detection' / 'frame1_color_camera_info.pkl'
        depth_camera_info_pkl_fpath = Path(__file__).parent / 'assets' / 'aruco_detection' / 'frame1_depth_camera_info.pkl'
        color_image_fpath = Path(__file__).parent / 'assets' / 'aruco_detection' / 'frame1_color_image.png'
        depth_image_fpath = Path(__file__).parent / 'assets' / 'aruco_detection' / 'frame1_depth_image.png'

        # load data
        with open(str(marker_info_yaml_fpath)) as f:
            marker_info = yaml.load(f, Loader=SafeLoader)
        color_image = cv2.imread(str(color_image_fpath))
        depth_image = cv2.imread(str(depth_image_fpath))
        with open(str(color_camera_info_pkl_fpath), 'rb') as inp:
            color_camera_info = pickle.load(inp)
        with open(str(depth_camera_info_pkl_fpath), 'rb') as inp:
            depth_camera_info = pickle.load(inp)
        depth_scale = 0.0010000000474974513

        # marker = stretch_pyfunmap.aruco.ArucoMarker(0, marker_info)
        # collection = stretch_pyfunmap.aruco.ArucoMarkerCollection(marker_info)
        # collection.update(rgb_image=np.zeros(shape=(1280, 720, 3)),
        #                   camera_info=?)
        self.assertTrue(True)

