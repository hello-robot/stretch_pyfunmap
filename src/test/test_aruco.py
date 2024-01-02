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
        link_name = 'camera_color_optical_frame'
        marker_info_yaml_fpath = Path(__file__).parent / 'assets' / 'aruco_detection' / 'aruco_marker_info.yaml'
        with open(str(marker_info_yaml_fpath)) as f:
            marker_info = yaml.load(f, Loader=SafeLoader)
        # 130 is a valid ID that exists in the marker info dictionary
        try:
            marker = stretch_pyfunmap.aruco.ArucoMarker(130, link_name, marker_info)
        except ValueError:
            self.fail("shouldn't have raised a ValueError when given valid ID 130")

        # 0 is not a ID that exists in the marker info dictionary
        with self.assertRaises(ValueError):
            marker = stretch_pyfunmap.aruco.ArucoMarker(0, link_name, marker_info)

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
        depth_image = cv2.imread(str(depth_image_fpath))[:,:,0] # TODO: use np.save and np.load instead of cv2.imwrite/imread for depth images
        with open(str(color_camera_info_pkl_fpath), 'rb') as inp:
            color_camera_info = pickle.load(inp)
        with open(str(depth_camera_info_pkl_fpath), 'rb') as inp:
            depth_camera_info = pickle.load(inp)
        depth_scale = 0.0010000000474974513
        link_name = 'camera_color_optical_frame'

        # run aruco detection
        collection = stretch_pyfunmap.aruco.ArucoMarkerCollection(marker_info, link_name)
        collection.update(rgb_image=color_image,
                          rgb_camera_info=color_camera_info,
                          depth_image=depth_image,
                          depth_camera_info=depth_camera_info,
                          depth_scale=depth_scale)

        collection.draw_markers(color_image)
        # cv2.imwrite('detections.png', color_image)
