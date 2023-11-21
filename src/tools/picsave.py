import cv2
import numpy as np
import stretch_pyfunmap.robot

robot = stretch_pyfunmap.robot.FunmapRobot()
frames = robot.head_cam.wait_for_frames()
color_frame = frames.get_color_frame()
color_image = np.asanyarray(color_frame.get_data())
cv2.imwrite('image.png', color_image)

robot.body.stop()