#!/usr/bin/env python3

import stretch_pyfunmap.navigate as nv
import stretch_pyfunmap.ros_max_height_image as rm

import rospy
import ros_numpy as rn
import stretch_pyfunmap.utils as utils
from std_srvs.srv import Trigger, TriggerRequest
from control_msgs.msg import FollowJointTrajectoryResult
from actionlib_msgs.msg import GoalStatus


class ROSFastSingleViewPlanner(nv.FastSingleViewPlanner):

    def __init__(self, debug_directory=None):
        if debug_directory is not None:
            self.debug_directory = debug_directory + 'fast_single_view_planner/'
            print('MoveBase __init__: self.debug_directory =', self.debug_directory)
        else:
            self.debug_directory = debug_directory

        # Define the volume of interest for planning using the current
        # view.

        # How far to look ahead.
        look_ahead_distance_m = 2.0
        # Robot's width plus a safety margin.
        look_to_side_distance_m = 1.3

        m_per_pix = 0.006
        pixel_dtype = np.uint8

        robot_head_above_ground = 1.13
        lowest_distance_below_ground = 0.03

        voi_height_m = robot_head_above_ground + lowest_distance_below_ground
        robot_front_x_m = -0.1
        voi_side_x_m = abs(robot_front_x_m) + look_ahead_distance_m
        voi_side_y_m = 2.0 * look_to_side_distance_m
        voi_axes = np.identity(3)
        voi_origin = np.array([robot_front_x_m, -voi_side_y_m/2.0, -lowest_distance_below_ground])
        self.frame_id = 'base_link'
        self.voi = rm.ROSVolumeOfInterest(self.frame_id, voi_origin, voi_axes, voi_side_x_m, voi_side_y_m, voi_height_m)
        self.max_height_im = rm.ROSMaxHeightImage(self.voi, m_per_pix, pixel_dtype)
        self.max_height_im.print_info()
        self.updated = False

    def check_line_path(self, end_xyz, end_frame_id, tf2_buffer, floor_mask=None):
        if self.updated:
            robot_xy_pix, robot_ang_rad, timestamp = self.max_height_im.get_robot_pose_in_image(tf2_buffer)
            robot_xya_pix = [robot_xy_pix[0], robot_xy_pix[1], robot_ang_rad]
            end_xy_pix = self.max_height_im.get_point_in_image(end_xyz, end_frame_id, tf2_buffer)[:2]

            debug = True
            if debug and (self.debug_directory is not None):
                # Save the new scan to disk.
                dirname = self.debug_directory + 'check_line_path/'
                filename = 'check_line_path_' + utils.create_time_string()
                print('FastSingleViewPlanner check_line_path : directory =', dirname)
                print('FastSingleViewPlanner check_line_path : filename =', filename)
                if not os.path.exists(dirname):
                    os.makedirs(dirname)
                self.max_height_im.save(dirname + filename)
                im = self.max_height_im.image.copy()
                rpix = np.int64(np.round(robot_xya_pix))
                radius = 20
                color = 255
                cv2.circle(im, tuple(rpix[:2]), radius, color, 2)
                rospy.loginfo('end_xy_pix = {0}'.format(end_xy_pix))
                epix = np.int64(np.round(end_xy_pix))
                cv2.circle(im, tuple(epix), radius, color, 2)
                cv2.imwrite(dirname + filename + '_with_start_and_end.png', im)

            check_result = na.check_line_path(self.max_height_im, robot_xya_pix, end_xy_pix, floor_mask=floor_mask)
            return check_result
        rospy.logerr('FastSingleViewPlanner.check_line_path called without first updating the scan with a pointcloud.')
        return False

    def plan_a_path(self, end_xyz, end_frame_id, tf2_buffer, floor_mask=None):
        if not self.updated:
            return None, None

        robot_xy_pix, robot_ang_rad, timestamp = self.max_height_im.get_robot_pose_in_image(tf2_buffer)
        robot_xya_pix = [robot_xy_pix[0], robot_xy_pix[1], robot_ang_rad]
        end_xy_pix = self.max_height_im.get_point_in_image(end_xyz, end_frame_id, tf2_buffer)[:2]

        debug = True
        if debug and (self.debug_directory is not None):
            # Save the new scan to disk.
            dirname = self.debug_directory + 'plan_a_path/'
            filename = 'plan_a_path_' + utils.create_time_string()
            print('FastSingleViewPlanner plan_a_path : directory =', dirname)
            print('FastSingleViewPlanner plan_a_path : filename =', filename)
            if not os.path.exists(dirname):
                os.makedirs(dirname)
            self.save_scan(dirname + filename)
            im = self.max_height_im.image.copy()
            rpix = np.int64(np.round(robot_xya_pix))
            radius = 20
            color = 255
            cv2.circle(im, tuple(rpix[:2]), radius, color, 2)
            rospy.loginfo('end_xy_pix = {0}'.format(end_xy_pix))
            epix = np.int64(np.round(end_xy_pix))
            cv2.circle(im, tuple(epix), radius, color, 2)
            cv2.imwrite(dirname + filename + '_with_start_and_end.png', im)

        line_segment_path, message = na.plan_a_path(self.max_height_im, robot_xya_pix, end_xy_pix, floor_mask=floor_mask)
        if line_segment_path is not None:
            output_frame = 'base_link'
            image_to_points_mat, ip_timestamp = self.max_height_im.get_image_to_points_mat(output_frame, tf2_buffer)
            path = [np.matmul(image_to_points_mat, np.array([p[0], p[1], 0.0, 1.0])) for p in line_segment_path]
            path = [[p[0], p[1], 0.0] for p in path]
            return path, 'base_link'

        return None, None

    def update(self, point_cloud_msg, tf2_buffer):
        self.max_height_im.clear()
        # cloud_time = point_cloud_msg.header.stamp
        cloud_frame = point_cloud_msg.header.frame_id
        point_cloud = rn.numpify(point_cloud_msg)
        only_xyz = False
        if only_xyz:
            xyz = rn.point_cloud2.get_xyz_points(point_cloud)
            self.max_height_im.from_points_with_tf2(xyz, cloud_frame, tf2_buffer)
        else:
            rgb_points = rn.point_cloud2.split_rgb_field(point_cloud)
            self.max_height_im.from_rgb_points_with_tf2(rgb_points, cloud_frame, tf2_buffer)
        # obstacle_im = self.max_height_im.image == 0
        self.updated = True

    def save_scan(self, filename):
        # Save the new scan to disk.
        self.max_height_im.save(filename)

    def publish_visualizations(self, voi_marker_pub, point_cloud_pub):
        marker = self.voi.get_ros_marker(duration=1000.0)
        voi_marker_pub.publish(marker)
        point_cloud = self.max_height_im.to_point_cloud()
        point_cloud_pub.publish(point_cloud)


class ROSMoveBase(nv.MoveBase):

    def __init__(self, node, debug_directory=None):
        super().__init__(node, debug_directory)
        self.node = self.robot
        self.local_planner = ROSFastSingleViewPlanner()
        self.unsuccessful_status = [GoalStatus.PREEMPTED, GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.LOST]

    def check_move_state(self, trajectory_client):
        at_goal = False
        unsuccessful_action = False
        state = trajectory_client.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo('Move succeeded!')
            # wait for the motion to come to a complete stop
            rospy.sleep(0.5)
            at_goal = True
        elif state in self.unsuccessful_status:
            rospy.loginfo('Move action terminated without success (state = {0}).'.format(state))
            unsuccessful_action = True
        return at_goal, unsuccessful_action
