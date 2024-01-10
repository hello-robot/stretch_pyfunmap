#!/usr/bin/env python3

import yaml
import numpy as np
import stretch_pyfunmap.mapping as ma
import stretch_pyfunmap.ros_max_height_image as rmhi
import stretch_pyfunmap.utils as utils

import rospy
import ros_numpy
import tf_conversions


class ROSHeadScan(ma.HeadScan):

    def __init__(self, node, max_height_im=None, voi_side_m=8.0, voi_origin_m=None):
        super().__init__(node, max_height_im, voi_side_m, voi_origin_m)
        rmhi.ROSMaxHeightImage.from_mhi(self.max_height_im) # convert MaxHeightImage to ROS
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
            cloud_time = self.node.point_cloud.header.stamp
            if (cloud_time is not None) and (cloud_time != prev_cloud_time) and (cloud_time >= settle_time):
                cloud_frame = self.node.point_cloud.header.frame_id
                point_cloud = ros_numpy.numpify(self.node.point_cloud)
                rgb_points = ros_numpy.point_cloud2.split_rgb_field(point_cloud)
                self.max_height_im.from_rgb_points_with_tf2(rgb_points, cloud_frame, self.node.tf2_buffer)
                num_point_clouds += 1
                prev_cloud_time = cloud_time
                
    def execute(self, head_tilt, far_left_pan, far_right_pan, num_pan_steps, capture_params, look_at_self=True):
        scan_start_time = rospy.get_time()

        pose = {'joint_head_pan': far_right_pan, 'joint_head_tilt': head_tilt}
        self.node.move_to_pose(pose)

        pan_left = np.linspace(far_right_pan, far_left_pan, num_pan_steps)

        for pan_ang in pan_left:
            pose = {'joint_head_pan': pan_ang}
            self.capture_point_clouds(pose, capture_params)

        # look at the ground right around the robot to detect any
        # nearby obstacles

        if look_at_self:
            # Attempt to pick a head pose that sees around the robot,
            # but doesn't see the mast, which can introduce noise.
            head_tilt = -1.2
            head_pan = 0.1
            pose = {'joint_head_pan': head_pan, 'joint_head_tilt': head_tilt}
            self.capture_point_clouds(pose, capture_params)

        scan_end_time = rospy.get_time()
        scan_duration = scan_end_time - scan_start_time
        rospy.loginfo(f'HeadScan.execute INFO: The head scan took {scan_duration} seconds.')

        #####################################
        # record robot pose information and potentially useful transformations
        self.robot_xy_pix, self.robot_ang_rad, self.timestamp = self.max_height_im.get_robot_pose_in_image(self.node.tf2_buffer)

        # Should only need three of these transforms, since the other
        # three should be obtainable via matrix inversion. Variation
        # in time could result in small differences due to encoder
        # noise.
        self.base_link_to_image_mat, timestamp = self.max_height_im.get_points_to_image_mat('base_link', self.node.tf2_buffer)
        self.base_link_to_map_mat, timestamp = utils.get_p1_to_p2_matrix('base_link', 'map', self.node.tf2_buffer)
        self.image_to_map_mat, timestamp = self.max_height_im.get_image_to_points_mat('map', self.node.tf2_buffer)
        self.image_to_base_link_mat, timestamp = self.max_height_im.get_image_to_points_mat('base_link', self.node.tf2_buffer)
        self.map_to_image_mat, timestamp = self.max_height_im.get_points_to_image_mat('map', self.node.tf2_buffer)
        self.map_to_base_mat, timestamp = utils.get_p1_to_p2_matrix('map', 'base_link', self.node.tf2_buffer)

        self.make_robot_mast_blind_spot_unobserved()
        self.make_robot_footprint_unobserved()

    def save(self, base_filename, save_visualization=True):
        rospy.loginfo(f'HeadScan.save INFO: Saving to base_filename = {base_filename}')
        # save scan to disk
        max_height_image_base_filename = base_filename + '_mhi'
        self.max_height_im.save(max_height_image_base_filename)

        if "tolist" in dir(self.robot_ang_rad):
            robot_ang_rad = self.robot_ang_rad.tolist()
        else:
            robot_ang_rad = self.robot_ang_rad
        data = {'max_height_image_base_filename' : max_height_image_base_filename,
                'robot_xy_pix' : self.robot_xy_pix.tolist(),
                'robot_ang_rad' : robot_ang_rad,
                'timestamp' : {'secs':self.timestamp.secs, 'nsecs':self.timestamp.nsecs},
                'base_link_to_image_mat' : self.base_link_to_image_mat.tolist(),
                'base_link_to_map_mat' : self.base_link_to_map_mat.tolist(),
                'image_to_map_mat' : self.image_to_map_mat.tolist(),
                'image_to_base_link_mat' : self.image_to_base_link_mat.tolist(),
                'map_to_image_mat' : self.map_to_image_mat.tolist(),
                'map_to_base_mat' : self.map_to_base_mat.tolist()}

        with open(base_filename + '.yaml', 'w') as fid:
            yaml.dump(data, fid)
