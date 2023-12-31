import unittest

import stretch_pyfunmap.aruco


class TestAruco(unittest.TestCase):

    def test_temp(self):
        """TODO
        """
        marker_info = {
            'default': {
                'length_mm': 20.0,
                'use_rgb_only': True
            }
        }
        marker = stretch_pyfunmap.aruco.ArucoMarker(0, marker_info)
        collection = stretch_pyfunmap.aruco.ArucoMarkerCollection(marker_info)
        self.assertTrue(True)

