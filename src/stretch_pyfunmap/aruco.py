#!/usr/bin/env python3

import cv2
import math
import numpy as np


class ArucoMarker:

    def __init__(self, aruco_id, marker_info, show_debug_images=False):
        self.show_debug_images = show_debug_images

        self.aruco_id = aruco_id
        colormap = cv2.COLORMAP_HSV
        offset = 0
        i = (offset + (self.aruco_id * 29)) % 255
        image = np.uint8([[[i]]])
        id_color_image = cv2.applyColorMap(image, colormap)
        bgr = id_color_image[0,0]
        self.id_color = [bgr[2], bgr[1], bgr[0]]

        self.frame_id = 'camera_color_optical_frame'
        self.info = marker_info.get(str(self.aruco_id), None)

        if self.info is None:
            self.info = marker_info['default']
        self.length_of_marker_mm = self.info['length_mm']
        self.use_rgb_only = self.info['use_rgb_only']

        # Distance beyond which the depth image depth will be
        # used to estimate the marker position.
        # 280mm is the minimum depth for the D435i at 1280x720 resolution
        self.min_z_to_use_depth_image = 0.28 + (self.length_of_marker_mm/1000.0)

        self.frame_number = None
        self.timestamp = None
        self.plane = None
        self.points_array = None
        self.ready = False

        self.x_axis = None
        self.y_axis = None
        self.z_axis = None
        self.used_depth_image = False
        self.broadcasted = False


    def get_marker_point_cloud(self):
        return self.points_array

    def get_plane_fit_point_cloud(self):
        if self.plane is None:
            return None
        origin = np.array(self.marker_position)
        side_length = self.length_of_marker_mm/1000.0
        sample_spacing = 0.001
        points = self.plane.get_points_on_plane(origin, side_length, sample_spacing)
        return points

    def update_marker_point_cloud(self, aruco_depth_estimate):
        if (not self.ready) or (self.depth_image is None):
            self.points_array = None

        c = self.center
        mn = self.min_dists
        mx = self.max_dists
        corners = self.corners
        id_num = self.aruco_id

        # Find rectangle that encloses the ArUco detected corners.
        left = int(math.floor(c[0] + mn[0]))
        right = int(math.ceil(c[0] + mx[0]))
        top = int(math.floor(c[1] + mn[1]))
        bottom = int(math.ceil(c[1] + mx[1]))

        # Crop rectangle of depth map corresponding with detected ArUco corners.
        depth_crop = self.depth_image[top : bottom, left : right]

        # Create a mask corresponding with the polygon defined by the
        # ArUco corners.
        mask_crop = np.zeros_like(depth_crop, np.uint8)
        crop_poly_points = np.array(corners) - [left, top]
        crop_poly_points = np.round(crop_poly_points).astype(np.int32)
        # TODO: Check how this treats the boundary pixels. Will it
        # include or exclude the corners? Should it include or exclude
        # the corners?
        cv2.fillConvexPoly(mask_crop, crop_poly_points, 255)

        # Create array with pixel coordinates for the cropped region of the depth map.
        coord_crop = np.mgrid[top : bottom : 1, left : right : 1]

        # Decompose the camera matrix.
        camera_matrix = np.reshape(self.camera_info.K, (3,3))
        f_x = camera_matrix[0,0]
        c_x = camera_matrix[0,2]
        f_y = camera_matrix[1,1]
        c_y = camera_matrix[1,2]

        # Convert the points in the cropped rectangle of the depth
        # image to 3D points in meters using the camera matrix.
        z = depth_crop/1000.0
        x = ((coord_crop[1] - c_x) / f_x) * z
        y = ((coord_crop[0] - c_y) / f_y) * z

        # Filter the points based on their depth to remove extreme
        # outliers. Without this, there is a tendency for some depths
        # to read 0 or near 0 (by the camera) and some depths to be
        # far away (e.g., on the floor), which can result in plane
        # fitting problems.

        # TODO: Better handle situations when the cropped rectangle
        # contains no reasonable depth values.

        # First, weakly filter the points using the RGB only depth
        # estimate from the ArUco code. This is a weak filter due to
        # the estimate's sensitivity to corner detection errors.
        marker_length_m = self.length_of_marker_mm/1000.0
        min_z = aruco_depth_estimate - (6 * marker_length_m)
        max_z = aruco_depth_estimate + (6 * marker_length_m)
        mask_z = (z > min_z) & (z < max_z)

        # Second, filter for depths that are within one marker length
        # away from the median depth.
        remaining_z = z[mask_z]
        if len(remaining_z) > 0:
            median_z = np.median(z[mask_z])
            min_z = median_z - marker_length_m
            max_z = median_z + marker_length_m
            mask_z = (z > min_z) & (z < max_z)

            # Combine the values into a numpy array with the following
            # structure: [[x0, y0, z0], [x1, y1, z1], ... ]
            d = np.dstack([x,y,z])
            s = d.shape
            points_array = d.reshape((s[0]*s[1],3))

            # Only use the points that are within the polygon formed by
            # the ArUco corners and fall within a reasonable range of
            # depths.
            points_array = points_array[(mask_crop.flatten() > 0) & mask_z.flatten()]

            if self.show_debug_images:
                norm_depth_image = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
                cv2.imshow('depth_image', norm_depth_image)
                display_depth_crop = cv2.normalize(depth_crop, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
                cv2.imshow('Cropped Depth', display_depth_crop)
                display_mask_crop = cv2.normalize(mask_crop, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
                cv2.imshow('Cropped Mask', display_mask_crop)
        else:
            points_array = np.empty((0,3), dtype=np.float64)

        self.points_array = points_array


    def update(self, corners, timestamp, frame_number, camera_info, depth_image=None):
        self.ready = True
        self.corners = corners
        self.timestamp = timestamp
        self.frame_number = frame_number
        self.camera_info = camera_info
        self.depth_image = depth_image
        self.camera_matrix = np.reshape(self.camera_info.K, (3,3))
        self.distortion_coefficients = np.array(self.camera_info.D)
        rvecs = np.zeros((len(self.corners), 1, 3), dtype=np.float64)
        tvecs = np.zeros((len(self.corners), 1, 3), dtype=np.float64)
        points_3D = np.array([
            (-self.length_of_marker_mm / 2, self.length_of_marker_mm / 2, 0),
            (self.length_of_marker_mm / 2, self.length_of_marker_mm / 2, 0),
            (self.length_of_marker_mm / 2, -self.length_of_marker_mm / 2, 0),
            (-self.length_of_marker_mm / 2, -self.length_of_marker_mm / 2, 0),
        ])
        for marker_num in range(len(self.corners)):
            unknown_variable, rvecs_ret, tvecs_ret = cv2.solvePnP(objectPoints=points_3D, imagePoints=self.corners[marker_num], cameraMatrix=self.camera_matrix, distCoeffs=self.distortion_coefficients)
            rvecs[marker_num][:] = np.transpose(rvecs_ret)
            tvecs[marker_num][:] = np.transpose(tvecs_ret)
        self.aruco_rotation = rvecs[0][0]
        # Convert ArUco position estimate to be in meters.
        self.aruco_position = tvecs[0][0]/1000.0
        aruco_depth_estimate = self.aruco_position[2]

        self.center = np.average(self.corners, axis=1).flatten()
        self.min_dists = np.min((self.corners - self.center), axis=1).flatten()
        self.max_dists = np.max((self.corners - self.center), axis=1).flatten()

        if (self.depth_image is not None) and (aruco_depth_estimate > self.min_z_to_use_depth_image) and (not self.use_rgb_only):
            only_use_rgb = False

            # Find suitable 3D points within the polygon defined by
            # the ArUco detected corners. If there are too few points,
            # do not proceed with fitting a plane and instead only use
            # the RGB image to perform the 3D estimation using the
            # ArUco code.

            self.update_marker_point_cloud(aruco_depth_estimate)
            num_points = self.points_array.shape[0]
            min_number_of_points_for_plane_fitting = 16
            if num_points < min_number_of_points_for_plane_fitting:
                print('WARNING: There are too few points from the depth image for plane fitting, so only using the RGB ArUco estimate. number of points =', num_points)
                only_use_rgb = True
        else:
            only_use_rgb = True

        if not only_use_rgb:
            # Use the depth image and the RGB image to estimate the
            # marker's pose. The RGB image is used to identify the
            # marker and detect the corners of the marker. The depth
            # image is used to fit a plane to the marker. The ArUco
            # detected corners are then projected onto this fit plane
            # and their projections are used to estimate the marker's
            # position and the coordinate system on the plane (x axis
            # and y axis). The z axis is normal to the fit plane.

            self.used_depth_image = True

            self.plane = fp.FitPlane()
            self.plane.fit_svd(self.points_array, verbose=False)

            # Find the points on the fit plane corresponding with the
            # four ArUco corners. Then, use the mean of the 4 points
            # as the 3D center for the marker.
            d = self.plane.d
            n = self.plane.n
            f_x = self.camera_matrix[0,0]
            c_x = self.camera_matrix[0,2]
            f_y = self.camera_matrix[1,1]
            c_y = self.camera_matrix[1,2]

            def pix_to_plane(pix_x, pix_y):
                z = 1.0
                x = ((pix_x - c_x) / f_x) * z
                y = ((pix_y - c_y) / f_y) * z
                point = np.array([x, y, z])
                ray = point/np.linalg.norm(point)
                point = ((d / np.matmul(n.transpose(), ray)) * ray).flatten()
                return point

            # "markerCorners is the list of corners of the detected markers. For
            # each marker, its four corners are returned in their original order
            # (which is clockwise starting with top left). So, the first corner is
            # the top left corner, followed by the top right, bottom right and
            # bottom left."
            # https://docs.opencv.org/4.0.1/d5/dae/tutorial_aruco_detection.html
            #
            # y axis points to the top of the marker
            # x axis points to the right of the marker
            # z axis points out of the marker (normal to the marker)
            corner_points = []
            total_corner = np.array([0.0, 0.0, 0.0])
            for (pix_x, pix_y) in self.corners[0]:
                corner_point = pix_to_plane(pix_x, pix_y)
                total_corner += corner_point
                corner_points.append(corner_point)
            self.marker_position = total_corner / 4.0

            # Use the corners on the fit plane to estimate the x and y
            # axes for the marker.
            top_left, top_right, bottom_right, bottom_left = corner_points

            y_axis = (top_left + top_right) - (bottom_left + bottom_right)
            y_length = np.linalg.norm(y_axis)
            if y_length > 0.0:
                y_axis = y_axis/y_length
            else:
                y_axis = None

            x_axis = (top_right + bottom_right) - (top_left + bottom_left)
            x_length = np.linalg.norm(x_axis)
            if x_length > 0.0:
                x_axis = x_axis/x_length
            else:
                x_axis = None

            plane_normal = self.plane.get_plane_normal()
            R = np.identity(4)
            R[:3,:3] = cv2.Rodrigues(self.aruco_rotation)[0]

            if x_axis is not None:
                old_x_axis = np.reshape(x_axis, (3,1))
            else:
                old_x_axis = np.reshape(R[:3,0], (3,1))
            if y_axis is not None:
                old_y_axis = np.reshape(y_axis, (3,1))
            else:
                old_y_axis = np.reshape(R[:3,1], (3,1))

            # The following methods directly use the z axis from the
            # plane fit.
            new_z_axis = plane_normal
            if (x_axis is not None) and (y_axis is None):
                # If x_axis found, but not y_axis.
                new_x_axis = old_x_axis - (np.matmul(new_z_axis.transpose(), old_x_axis) * new_z_axis)
                new_x_axis = new_x_axis/np.linalg.norm(new_x_axis)
                new_y_axis = np.reshape(np.cross(new_z_axis.flatten(), new_x_axis.flatten()), (3,1))
            elif (x_axis is None) and (y_axis is not None):
                # If y_axis found, but not x_axis.
                new_y_axis = old_y_axis - (np.matmul(new_z_axis.transpose(), old_y_axis) * new_z_axis)
                new_y_axis = new_y_axis/np.linalg.norm(new_y_axis)
                new_x_axis = np.reshape(np.cross(new_y_axis.flatten(), new_z_axis.flatten()), (3,1))
            else:
                # Either both x_axis and y_axis were found, or neither.
                if (x_axis is None) and (y_axis is None):
                    print('WARNING: The detected ArUco corners did not project to reasonable 3D points on the fit plane.')
                    print('         self.corners[0] =', self.corners[0])

                # Attempt to reduce bias due to selecting one of the
                # old axes by averaging the results from both axes.
                new_y_axis_1 = old_y_axis - (np.matmul(new_z_axis.transpose(), old_y_axis) * new_z_axis)
                new_y_axis_1 = new_y_axis_1/np.linalg.norm(new_y_axis_1)

                new_x_axis_1 = np.reshape(np.cross(new_y_axis_1.flatten(), new_z_axis.flatten()), (3,1))

                new_x_axis_2 = old_x_axis - (np.matmul(new_z_axis.transpose(), old_x_axis) * new_z_axis)
                new_x_axis_2 = new_x_axis_2/np.linalg.norm(new_x_axis_2)

                new_x_axis = (new_x_axis_1 + new_x_axis_2)/2.0
                new_x_axis = new_x_axis/np.linalg.norm(new_x_axis)
                new_y_axis = np.reshape(np.cross(new_z_axis.flatten(), new_x_axis.flatten()), (3,1))

            self.x_axis = new_x_axis.flatten()
            self.y_axis = new_y_axis.flatten()
            self.z_axis = new_z_axis.flatten()

            R[:3,0] = self.x_axis
            R[:3,1] = self.y_axis
            R[:3,2] = self.z_axis

            self.marker_quaternion = quaternion_from_matrix(R)
        else:
            # Only use the RGB image for the marker pose
            # estimate. ArUco code performs this estimation.

            self.used_depth_image = False
            self.marker_position = self.aruco_position
            R = np.identity(4)
            R[:3,:3] = cv2.Rodrigues(self.aruco_rotation)[0]
            self.marker_quaternion = quaternion_from_matrix(R)
            self.x_axis = R[:3,0]
            self.y_axis = R[:3,1]
            self.z_axis = R[:3,2]

        self.broadcasted = False
        self.ready = True


    def get_marker_poly(self):
        poly_points = np.array(corners)
        poly_points = np.round(poly_points).astype(np.int32)
        return poly_points

    def draw_marker_poly(self, image):
        poly_points = self.get_marker_poly()
        cv2.fillConvexPoly(image, poly_points, (255, 0, 0))


class ArucoMarkerCollection:
    def __init__(self, marker_info, show_debug_images=False):
        self.show_debug_images = show_debug_images

        self.marker_info = marker_info
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_detection_parameters = cv2.aruco.DetectorParameters()
        self.aruco_detection_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_detection_parameters.cornerRefinementWinSize = 2
        self.collection = {}
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_detection_parameters)
        self.frame_number = 0

    def __iter__(self):
        # iterates through currently visible ArUco markers
        keys = self.collection.keys()
        for k in keys:
            marker = self.collection[k]
            if marker.frame_number == self.frame_number:
                yield marker

    def draw_markers(self, image):
        return self.detector.drawDetectedMarkers(image, self.aruco_corners, self.aruco_ids)

    def update(self, rgb_image, camera_info, depth_image=None, timestamp=None):
        self.frame_number += 1
        self.timestamp = timestamp
        self.rgb_image = rgb_image
        self.camera_info = camera_info
        self.depth_image = depth_image
        self.gray_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2GRAY)
        image_height, image_width = self.gray_image.shape
        self.aruco_corners, self.aruco_ids, aruco_rejected_image_points = self.detector.detectMarkers(self.gray_image)
        if self.aruco_ids is None:
            num_detected = 0
        else:
            num_detected = len(self.aruco_ids)

        if self.aruco_ids is not None:
            for corners, aruco_id in zip(self.aruco_corners, self.aruco_ids):
                aruco_id = int(aruco_id)
                marker = self.collection.get(aruco_id, None)
                if marker is None:
                    new_marker = ArucoMarker(aruco_id, self.marker_info, self.show_debug_images)
                    self.collection[aruco_id] = new_marker

                self.collection[aruco_id].update(corners, self.timestamp, self.frame_number, self.camera_info, self.depth_image)
