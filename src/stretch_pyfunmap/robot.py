#!/usr/bin/env python3

import sys
import cv2
import time
import pathlib
import numpy as np
from urchin import URDF
import pyrealsense2 as rs
import stretch_body.robot
import stretch_body.hello_utils as hu

import stretch_pyfunmap.mapping as ma
import stretch_pyfunmap.utils as utils


class FunmapRobot:

    def __init__(self, body=None, head_cam=None, autoexposure_timeout=3.0):
        # setup Stretch's body
        self.body = body
        if self.body is None:
            self.body = stretch_body.robot.Robot()
            did_start = self.body.startup()
            if not did_start:
                print('CRITICAL: hardware failed to initialize completely')
                self.body.stop()
                sys.exit(1)
            is_runstopped = self.body.status['pimu']['runstop_event']
            if is_runstopped:
                print('CRITICAL: unable to create FunmapRobot while Stretch is runstopped')
                self.body.stop()
                sys.exit(1)
        if not self.body.is_homed():
            self.body.home()

        # setup Stretch's head camera
        self.head_cam = head_cam
        self._pc_creator = rs.pointcloud()
        if self.head_cam is None:
            self.head_cam = rs.pipeline()
            fps = 15
            resolution_color = (1280, 720) # (1920, 1080), (1280, 720), (640, 480)
            resolution_depth = (1280, 720) # options: (1280, 720), (640, 480)
            config = rs.config()
            config.enable_stream(rs.stream.color, resolution_color[0], resolution_color[1], rs.format.bgr8, fps)
            config.enable_stream(rs.stream.depth, resolution_depth[0], resolution_depth[1], rs.format.z16, fps)
            self.head_cam.start(config)

            # Warm up camera's auto exposure
            start = time.time()
            print(f'FunmapRobot.__init__ DEBUG: waiting {autoexposure_timeout} seconds for head cam autoexposure to adjust')
            while (time.time() - start < autoexposure_timeout):
                self.head_cam.wait_for_frames()

        # setup Stretch's urdf
        urdf_path = str((pathlib.Path(hu.get_fleet_directory()) / 'exported_urdf' / 'stretch.urdf').absolute())
        self.urdf = URDF.load(urdf_path, lazy_load_meshes=True)

    def _get_current_configuration(self):
        def bound_range(name, value):
            return min(max(value, self.urdf.joint_map[name].limit.lower), self.urdf.joint_map[name].limit.upper)

        tool = self.body.end_of_arm.name
        if tool == 'tool_stretch_gripper':
            q_lift = bound_range('joint_lift', self.body.lift.status['pos'])
            q_arml = bound_range('joint_arm_l0', self.body.arm.status['pos'] / 4.0)
            q_yaw = bound_range('joint_wrist_yaw', self.body.end_of_arm.status['wrist_yaw']['pos'])
            q_pan = bound_range('joint_head_pan', self.body.head.status['head_pan']['pos'])
            q_tilt = bound_range('joint_head_tilt', self.body.head.status['head_tilt']['pos'])
            return {
                'joint_left_wheel': 0.0,
                'joint_right_wheel': 0.0,
                'joint_lift': q_lift,
                'joint_arm_l0': q_arml,
                'joint_arm_l1': q_arml,
                'joint_arm_l2': q_arml,
                'joint_arm_l3': q_arml,
                'joint_wrist_yaw': q_yaw,
                'joint_gripper_finger_left': 0.0,
                'joint_gripper_finger_right': 0.0,
                'joint_head_pan': q_pan,
                'joint_head_tilt': q_tilt
            }
        elif tool == 'tool_stretch_dex_wrist':
            q_lift = bound_range('joint_lift', self.body.lift.status['pos'])
            q_arml = bound_range('joint_arm_l0', self.body.arm.status['pos'] / 4.0)
            q_yaw = bound_range('joint_wrist_yaw', self.body.end_of_arm.status['wrist_yaw']['pos'])
            q_pitch = bound_range('joint_wrist_pitch', self.body.end_of_arm.status['wrist_pitch']['pos'])
            q_roll = bound_range('joint_wrist_roll', self.body.end_of_arm.status['wrist_roll']['pos'])
            q_pan = bound_range('joint_head_pan', self.body.head.status['head_pan']['pos'])
            q_tilt = bound_range('joint_head_tilt', self.body.head.status['head_tilt']['pos'])
            return {
                'joint_left_wheel': 0.0,
                'joint_right_wheel': 0.0,
                'joint_lift': q_lift,
                'joint_arm_l0': q_arml,
                'joint_arm_l1': q_arml,
                'joint_arm_l2': q_arml,
                'joint_arm_l3': q_arml,
                'joint_wrist_yaw': q_yaw,
                'joint_wrist_pitch': q_pitch,
                'joint_wrist_roll': q_roll,
                'joint_gripper_finger_left': 0.0,
                'joint_gripper_finger_right': 0.0,
                'joint_head_pan': q_pan,
                'joint_head_tilt': q_tilt
            }

    def get_transform(self, from_frame, to_frame):
        q_curr = self._get_current_configuration()
        fk_curr = self.urdf.link_fk(cfg=q_curr)

        # get frames w.r.t. base link
        baselink_to_fromframe = None
        baselink_to_toframe = None
        for l in self.urdf.links:
            if l.name == from_frame:
                baselink_to_fromframe = fk_curr[l]
            if l.name == to_frame:
                baselink_to_toframe = fk_curr[l]
        if from_frame == 'map':
            baselink_to_fromframe = np.eye(4)
        if to_frame == 'map':
            baselink_to_toframe = np.eye(4)

        # calculate to_frame w.r.t from_frame
        fromframe_to_toframe = np.linalg.inv(baselink_to_fromframe).dot(baselink_to_toframe)
        return fromframe_to_toframe

    def get_point_cloud(self):
        frames = self.head_cam.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        color_image = np.asanyarray(color_frame.get_data())
        # depth_image = np.asanyarray(depth_frame.get_data())

        self._pc_creator.map_to(color_frame)
        points = self._pc_creator.calculate(depth_frame)
        xyz = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
        texcoords = np.asanyarray(points.get_texture_coordinates()).view(np.float32).reshape(-1, 2)
        cw, ch = color_image.shape[:2][::-1]
        v_map = np.clip(texcoords[:,0] * cw + 1.5, 0, cw+1).astype(np.uint32)
        u_map = np.clip(texcoords[:,1] * ch + 1.5, 0, ch+1).astype(np.uint32)
        color_padded = np.pad(color_image, pad_width=[(1, 1),(1, 1),(0, 0)])
        rgb = color_padded[u_map, v_map]

        cloud_arr = np.zeros((points.size(),), dtype=[
                        ('x', np.float32),
                        ('y', np.float32),
                        ('z', np.float32),
                        ('r', np.uint8),
                        ('g', np.uint8),
                        ('b', np.uint8)])
        cloud_arr['x'] = xyz[:, 0]
        cloud_arr['y'] = xyz[:, 1]
        cloud_arr['z'] = xyz[:, 2]
        cloud_arr['r'] = rgb[:, 2]
        cloud_arr['g'] = rgb[:, 1]
        cloud_arr['b'] = rgb[:, 0]

        # filter out non-rgb points
        cloud_arr = cloud_arr[(cloud_arr['r'] != 0.) & (cloud_arr['g'] != 0.) & (cloud_arr['b'] != 0.)]

        cloud_time = time.time()
        cloud_frame = 'camera_color_optical_frame'
        return cloud_time, cloud_frame, cloud_arr

    def show_head_cam(self, hold=False, align=False, pointcloud=False, apply_clahe=False, depth_colormap=cv2.COLORMAP_OCEAN, imwrite=None):
        try:
            while True:
                # Get the latest frames from the camera
                frames = self.head_cam.wait_for_frames()
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue

                # Align color to depth
                if align:
                    aligner = rs.align(rs.stream.depth)
                    aligned = aligner.process(frames)
                    color_aligned_to_depth_frame = aligned.first(rs.stream.color)

                # Convert images to numpy arrays
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                if align:
                    color_aligned_to_depth_image = np.asanyarray(color_aligned_to_depth_frame.get_data())

                # Create pointcloud image
                if pointcloud:
                    pc_image = np.empty((depth_image.shape[0], depth_image.shape[1], 3), dtype=np.uint8)
                    pc_image.fill(100)
                    # utils.grid(pc_image, (0, 0.5, 1))
                    # depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()
                    # utils.frustum(pc_image, depth_intrinsics)
                    utils.pointcloud(pc_image, self.get_point_cloud()[2])
                    utils.axes(pc_image, utils.view([0, 0, 0]), utils.state.rotation, size=0.1, thickness=1)

                # Apply histogram equalization if desired
                if apply_clahe:
                    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
                    color_image_lab = cv2.cvtColor(color_image, cv2.COLOR_BGR2LAB)  # convert from BGR to LAB color space
                    color_image_l, color_image_a, color_image_b = cv2.split(color_image_lab)  # split on 3 different channels
                    color_image_l_equalized = clahe.apply(color_image_l)  # apply CLAHE to the L-channel
                    color_image_lab = cv2.merge((color_image_l_equalized, color_image_a, color_image_b))  # merge channels
                    color_image = cv2.cvtColor(color_image_lab, cv2.COLOR_LAB2BGR)  # convert from LAB to BGR

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=-0.04, beta=255.0), depth_colormap)

                # Rotate and flip images
                color_image = np.moveaxis(color_image, 0, 1)
                color_image = np.fliplr(color_image)
                depth_image = np.moveaxis(depth_image, 0, 1)
                depth_image = np.fliplr(depth_image)
                if align:
                    color_aligned_to_depth_image = np.moveaxis(color_aligned_to_depth_image, 0, 1)
                    color_aligned_to_depth_image = np.fliplr(color_aligned_to_depth_image)
                if pointcloud:
                    pc_image = np.moveaxis(pc_image, 0, 1)
                    pc_image = np.fliplr(pc_image)

                # Concatenate images horizontally
                if color_image.shape[0] != depth_image.shape[0]:
                    print('WARN: cannot show head cam if color & depth are of different resolutions')
                    return
                colorAndDepth_image = np.hstack((color_image, depth_image))
                if align:
                    colorAndDepth_image = np.hstack((colorAndDepth_image, color_aligned_to_depth_image))
                if pointcloud:
                    colorAndDepth_image = np.hstack((colorAndDepth_image, pc_image))

                # Write image instead of showing if imwrite path is set
                if imwrite:
                    cv2.imwrite(imwrite, colorAndDepth_image)
                    return

                # Show image in window
                cv2.namedWindow('Head Cam', cv2.WINDOW_AUTOSIZE)
                if pointcloud:
                    utils.state.total_image_shape = colorAndDepth_image.shape
                    cv2.setMouseCallback('Head Cam', utils.mouse_cb)
                cv2.imshow('Head Cam', colorAndDepth_image)
                wknum = 0 if hold else 1 # 0 -> infinite hold, 1 -> loop after 1ms
                wkret = cv2.waitKey(wknum)
                wkret = 0 if wkret < 0 else wkret
                if wkret:
                    cv2.destroyAllWindows()
                    return
        except KeyboardInterrupt:
            return

    def move_to_pose(self, pose, return_before_done=False, custom_contact_thresholds=False):
        if custom_contact_thresholds:
            raise Exception('FunmapRobot.move_to_pose ERROR: I dont support contact thresholds yet')
        print(f'Moving to {pose}')
        if 'joint_lift' in pose:
            self.body.lift.move_to(pose['joint_lift'])
        if 'wrist_extension' in pose:
            self.body.arm.move_to(pose['wrist_extension'])
        self.body.push_command()
        if 'joint_wrist_yaw' in pose:
            self.body.end_of_arm.move_to('wrist_yaw', pose['joint_wrist_yaw'])
        if 'joint_wrist_pitch' in pose:
            self.body.end_of_arm.move_to('wrist_pitch', pose['joint_wrist_pitch'])
        if 'joint_wrist_roll' in pose:
            self.body.end_of_arm.move_to('wrist_roll', pose['joint_wrist_roll'])
        if 'joint_head_pan' in pose:
            self.body.head.move_to('head_pan', pose['joint_head_pan'])
        if 'joint_head_tilt' in pose:
            self.body.head.move_to('head_tilt', pose['joint_head_tilt'])
        if 'joint_gripper_finger_left' in pose or 'joint_gripper_finger_right' in pose:
            print('FunmapRobot.move_to_pose WARN: gripper not executing')
        if not return_before_done:
            self.body.wait_command()

    def perform_head_scan(self, autoexposure_timeout=3.0):
        # Reduce occlusion from the arm and gripper
        self.body.stow()

        # Execute head scanning full procedure
        head_scanner = ma.HeadScan(self, voi_side_m=16.0)
        head_scanner.execute_full()
        return head_scanner
