import sys
import cv2
import numpy as np
import stretch_body.robot
import pyrealsense2 as rs


class FunmapRobot:

    def __init__(self, body=None, head_cam=None):
        # setup Stretch's body
        self.body = body
        if self.body is None:
            self.body = stretch_body.robot.Robot()
            did_start = self.body.startup()
            if not did_start:
                print('CRITICAL: hardware failed to initialize completely')
                self.body.stop()
                sys.exit(1)
        if not self.body.is_calibrated():
            self.body.home()

        # setup Stretch's head camera
        self.head_cam = head_cam
        if self.head_cam is None:
            self.head_cam = rs.pipeline()
            fps = 15
            resolution_color = (1280, 720) # (1920, 1080), (1280, 720), (640, 480)
            resolution_depth = (1280, 720) # options: (1280, 720), (640, 480)
            config = rs.config()
            config.enable_stream(rs.stream.color, resolution_color[0], resolution_color[1], rs.format.bgr8, fps)
            config.enable_stream(rs.stream.depth, resolution_depth[0], resolution_depth[1], rs.format.z16, fps)
            self.head_cam.start(config)

    def show_head_cam(self, hold=False, apply_clahe=False, depth_colormap=cv2.COLORMAP_OCEAN):
        try:
            while True:
                # Get the latest frames from the camera
                frames = self.head_cam.wait_for_frames()
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue

                # Convert images to numpy arrays
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())

                # Apply histogram equalization if desired
                if apply_local_histogram_equalization:
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

                # Concatenate images horizontally
                # Pad images if they are different heights
                pad_y = color_image.shape[0] - depth_image.shape[0]
                if pad_y > 0:
                    color_image_padded = color_image
                    depth_image_padded = np.pad(depth_image, ((pad_y, 0), (0, 0), (0, 0)), mode='constant', constant_values=0)
                else:
                    pad_y = abs(pad_y)
                    color_image_padded = np.pad(color_image, ((pad_y, 0), (0, 0), (0, 0)), mode='constant', constant_values=0)
                    depth_image_padded = depth_image
                # Tile the images
                colorAndDepth_image = np.hstack((color_image_padded, depth_image_padded))

                # Show image in window
                cv2.namedWindow('Head Cam', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('Head Cam', colorAndDepth_image)
                wknum = 0 if hold else 1 # 0 -> infinite hold, 1 -> loop after 1ms
                wkret = cv2.waitKey(wknum)
                wkret = 0 if wkret < 0 else wkret
                if wkret:
                    raise KeyboardInterrupt()
        except KeyboardInterrupt:
            return

    def head_scan(self):
        pass