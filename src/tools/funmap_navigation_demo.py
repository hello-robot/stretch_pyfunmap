#!/usr/bin/env python3

# Sample command line usage:
#   funmap_navigation_demo.py --load ./example_scan
#   python3 funmap_navigation_demo.py --load /home/hello-robot/stretch_user/debug/head_scans/head_scan_20240109235850
# See the parser arguments below for more command-line options.

import sys
import copy
import argparse
import numpy as np
import stretch_pyfunmap.robot
import stretch_pyfunmap.mapping as ma
import stretch_pyfunmap.navigate as nv
import stretch_pyfunmap.navigation_planning as na
from stretch_pyfunmap.mapping import display_head_scan

parser = argparse.ArgumentParser(description='Tool to demonstrate FUNMAP navigation.')
parser.add_argument("--load", nargs="?", type=str, const="",
                    help="Load head scan from given filepath. Use the funmap_head_scan_visualizer.py tool to create new head scans.")
parser.add_argument("--voi-side", type=float, default=8.0,
                    help="The length in meters for the map volume of interest.")
args, _ = parser.parse_known_args()

robot = stretch_pyfunmap.robot.FunmapRobot()
stdout = sys.stdout
with open('/dev/null', 'w') as sys.stdout:
    robot.body.stow() # Reduce occlusion from the arm and gripper
sys.stdout = stdout

# Get head scan
if args.load:
    # head_scanner = ma.HeadScan.from_file(args.load) # TODO
    raise NotImplementedError('--load flag not yet implemented')
else:
    print("Creating a map...")
    head_scanner = ma.HeadScan(robot, voi_side_m=args.voi_side)
    head_scanner.execute_full()

# User selects a desired base pose
input("""
Press enter. In the window that appears, you will see a MHI map
and the robot's pose marked in red. Mouse over the image to a
desired base goal, note the pixel x & y, and multiply them by 2.
Press 'q' to close the window, and enter them in the next prompt.
""")
display_head_scan('Head Scan', head_scanner, scale_divisor=2)
base_goal_str = input("""Enter desired goal as xya tuple (e.g. "(1000, 1000, 0.0)" where angle is in radians): """)
base_goal = eval(base_goal_str)

# Plan a path
start_xya_pix = [head_scanner.robot_xy_pix[0], head_scanner.robot_xy_pix[1], head_scanner.robot_ang_rad]
end_xy_pix = [base_goal[0], base_goal[1]]
line_segment_path, message = na.plan_a_path(head_scanner.max_height_im, start_xya_pix, end_xy_pix)
if line_segment_path is None:
    print(message)
    sys.exit(1)

# Get user confirmation
input("""
Press enter. In the window that appears, you will see the robot's
pose, the goal pose marked in green, and path between them that
the robot will follow. Press 'q' to close. If the planned path
looks okay, provide confirmation to proceed in the next prompt.
""")
mhi_image_backup = np.copy(head_scanner.max_height_im.image)
na.draw_line_segment_path(head_scanner.max_height_im.image, line_segment_path)
display_head_scan('Planned Path', head_scanner, scale_divisor=2, robot_xya_pix_list=[list(base_goal)])
head_scanner.max_height_im.image = mhi_image_backup
yn_str = input("""Proceed? (y/n): """)
if yn_str not in ["y", "Y", "yes", "YES"]:
    print("Confirmation not provided. Exiting.")
    sys.exit(1)

# Translate path
rotations = []
translations = []
curr_ang = head_scanner.robot_ang_rad
map_to_voi_mat = head_scanner.max_height_im.voi.get_points_to_voi_matrix(points_to_frame_id_mat=robot.get_transform(head_scanner.max_height_im.voi.frame_id, 'map'))
voi_to_map_mat = np.linalg.inv(map_to_voi_mat)
image_to_map_mat = head_scanner.max_height_im.get_image_to_points_mat(voi_to_map_mat)
for p0_pix, p1_pix in zip(line_segment_path, line_segment_path[1:]):
    p0 = (image_to_map_mat @ np.array([p0_pix[0], p0_pix[1], 0.0, 1.0]))[:2]
    p1 = (image_to_map_mat @ np.array([p1_pix[0], p1_pix[1], 0.0, 1.0]))[:2]
    travel_vector = p1 - p0
    travel_dist = np.linalg.norm(travel_vector)
    travel_ang = np.arctan2(travel_vector[1], travel_vector[0])
    turn_ang = utils.angle_diff_rad(travel_ang, curr_ang)
    curr_ang = travel_ang
    print(turn_ang, travel_dist)

robot.body.stop()
