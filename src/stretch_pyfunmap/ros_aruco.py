#!/usr/bin/env python3

import numpy as np
import stretch_pyfunmap.aruco as ar

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class ROSArucoMarker(ar.ArucoMarker):

    def __init__(self, aruco_id, marker_info, show_debug_images=False):
        super().__init__(aruco_id, marker_info, show_debug_images=False)

        self.marker = Marker()
        self.marker.type = self.marker.CUBE
        self.marker.action = self.marker.ADD
        self.marker.lifetime = rospy.Duration(0.2)
        self.marker.text = self.info['name']

    def broadcast_tf(self, tf_broadcaster, force_redundant=False):
        # Create TF frame for the marker. By default, only broadcast a
        # single time after an update.
        if (not self.broadcasted) or force_redundant:
            tf_broadcaster.sendTransform(self.marker_position, self.marker_quaternion, self.timestamp, self.marker.text, self.frame_id)
            self.broadcasted = True

    def get_ros_marker(self):
        if not self.ready:
            return None

        self.marker.header.frame_id = self.frame_id
        self.marker.header.stamp = self.timestamp
        self.marker.id = self.aruco_id

        # scale of 1,1,1 would result in a 1m x 1m x 1m cube
        self.marker.scale.x = self.length_of_marker_mm/1000.0
        self.marker.scale.y = self.length_of_marker_mm/1000.0
        self.marker.scale.z = 0.005 # half a centimeter tall

        # make as bright as possible
        den = float(np.max(self.id_color))
        self.marker.color.r = self.id_color[2]/den
        self.marker.color.g = self.id_color[1]/den
        self.marker.color.b = self.id_color[0]/den
        self.marker.color.a = 0.33

        self.marker.pose.position.x = self.marker_position[0]
        self.marker.pose.position.y = self.marker_position[1]
        self.marker.pose.position.z = self.marker_position[2]

        q = self.marker_quaternion
        self.marker.pose.orientation.x = q[0]
        self.marker.pose.orientation.y = q[1]
        self.marker.pose.orientation.z = q[2]
        self.marker.pose.orientation.w = q[3]

        return self.marker


    def create_axis_marker(self, axis, id_num, rgba=None, name=None):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.timestamp
        marker.id = id_num
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration(1.0)
        if name is not None:
            marker.text = name
        axis_arrow = {'head_diameter': 0.005,
                      'shaft_diameter': 0.003,
                      'head_length': 0.003,
                      'length': 0.02}
        # "scale.x is the shaft diameter, and scale.y is the
        # head diameter. If scale.z is not zero, it specifies
        # the head length." -
        # http://wiki.ros.org/rviz/DisplayTypes/Marker#Arrow_.28ARROW.3D0.29
        marker.scale.x = axis_arrow['shaft_diameter']
        marker.scale.y = axis_arrow['head_diameter']
        marker.scale.z = axis_arrow['head_length']

        if rgba is None:
            # make as bright as possible
            den = float(np.max(self.id_color))
            marker.color.r = self.id_color[2]/den
            marker.color.g = self.id_color[1]/den
            marker.color.b = self.id_color[0]/den
            marker.color.a = 0.33
        else:
            c = marker.color
            c.r, c.g, c.b, c.a = rgba

        start_point = Point()
        x = self.marker_position[0]
        y = self.marker_position[1]
        z = self.marker_position[2]
        start_point.x = x
        start_point.y = y
        start_point.z = z
        end_point = Point()
        length = axis_arrow['length']
        end_point.x = x + (axis[0] * length)
        end_point.y = y + (axis[1] * length)
        end_point.z = z + (axis[2] * length)
        marker.points = [start_point, end_point]
        return marker


    def get_ros_z_axis_marker(self):
        if not self.ready:
            return None

        if self.plane is None:
            return None

        if self.used_depth_image and (self.z_axis is not None):
            id_num = 3 * self.aruco_id
            return self.create_axis_marker(self.z_axis, id_num, rgba=None)
        else:
            return None

    def get_ros_axes_markers(self):
        markers = []

        if not self.ready:
            return markers

        # ROS color convention
        # x axis is red
        # y axis is green
        # z axis is blue

        base_name = self.info['name']

        if self.z_axis is not None:
            id_num = 3 * self.aruco_id
            rgba = [0.0, 0.0, 1.0, 0.33]
            name = base_name = '_z_axis'
            markers.append(self.create_axis_marker(self.z_axis, id_num, rgba, name))
        if self.x_axis is not None:
            id_num = (3 * self.aruco_id) + 1
            rgba = [1.0, 0.0, 0.0, 0.33]
            name = base_name = '_x_axis'
            markers.append(self.create_axis_marker(self.x_axis, id_num, rgba, name))
        if self.y_axis is not None:
            id_num = (3 * self.aruco_id) + 2
            rgba = [0.0, 1.0, 0.0, 0.33]
            name = base_name = '_y_axis'
            markers.append(self.create_axis_marker(self.y_axis, id_num, rgba, name))

        return markers


class ROSArucoMarkerCollection(ar.ArucoMarkerCollection):

    def broadcast_tf(self, tf_broadcaster):
        # Create TF frames for each of the markers. Only broadcast each
        # marker a single time after it has been updated.
        for key in self.collection:
            marker = self.collection[key]
            marker.broadcast_tf(tf_broadcaster)

    def get_ros_marker_array(self):
        marker_array = MarkerArray()
        for key in self.collection:
            marker = self.collection[key]
            if marker.frame_number == self.frame_number:
                ros_marker = marker.get_ros_marker()
                marker_array.markers.append(ros_marker)
        return marker_array

    def get_ros_axes_array(self, include_z_axes=True, include_axes=True):
        marker_array = MarkerArray()
        for key in self.collection:
            marker = self.collection[key]
            if marker.frame_number == self.frame_number:
                if include_z_axes:
                    ros_z_axis_marker = marker.get_ros_z_axis_marker()
                    if ros_z_axis_marker is not None:
                        marker_array.markers.append(ros_z_axis_marker)
                if include_axes:
                    ros_axes_markers= marker.get_ros_axes_markers()
                    marker_array.markers.extend(ros_axes_markers)
        return marker_array