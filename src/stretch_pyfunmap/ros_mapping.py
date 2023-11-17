#!/usr/bin/env python3

import stretch_pyfunmap.mapping as ma
import stretch_pyfunmap.ros_max_height_image as rmhi

import rospy
import ros_numpy
import tf_conversions
import hello_helpers.hello_misc as hm
from actionlib_msgs.msg import GoalStatus


class ROSHeadScan(ma.HeadScan):

    def __init__(self, node, max_height_im=None, voi_side_m=8.0, voi_origin_m=None):
        super().__init__(node, max_height_im, voi_side_m, voi_origin_m)
        rmhi.ROSMaxHeightImage.from_mhi(self.max_height_im) # convert MaxHeightImage to ROS # TODO: implement from_mhi()
        self.node = self.robot

    def capture_point_clouds(self, pose, capture_params):
        head_settle_time = capture_params['head_settle_time']
        num_point_clouds_per_pan_ang = capture_params['num_point_clouds_per_pan_ang']
        time_between_point_clouds = capture_params['time_between_point_clouds']
        fast_scan = capture_params.get('fast_scan', False)
        if fast_scan:
            num_point_clouds_per_pan_ang = 1

        self.node.move_to_pose(pose)
        rospy.sleep(head_settle_time)
        settle_time = rospy.Time.now()

        # Consider using time stamps to make decisions, instead of
        # hard coded sleep times, as found in the head calibration
        # data collection code. The main issue is that the robot
        # needs time to mechanically settle in addition to sensor
        # timing considerations.
        prev_cloud_time = None
        num_point_clouds = 0
        while num_point_clouds < num_point_clouds_per_pan_ang:
            rospy.sleep(time_between_point_clouds)
            cloud_time = self.node.point_cloud.stamp
            cloud_frame = self.node.point_cloud.header.frame_id
            point_cloud = ros_numpy.numpify(self.node.point_cloud)
            if (cloud_time is not None) and (cloud_time != prev_cloud_time) and (cloud_time >= settle_time):
                rgb_points = ros_numpy.point_cloud2.split_rgb_field(point_cloud)
                self.max_height_im.from_rgb_points_with_tf2(rgb_points, cloud_frame, self.node.tf2_buffer)
                num_point_clouds += 1
                prev_cloud_time = cloud_time
                

