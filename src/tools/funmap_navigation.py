#!/usr/bin/env python3

# Sample command line usage:
#   funmap_navigation.py --load ./example_scan
# See the parser arguments below for more command-line options.

import argparse
import stretch_pyfunmap.robot
import stretch_pyfunmap.mapping as ma
import stretch_pyfunmap.navigate as nv
from stretch_pyfunmap.mapping import display_head_scan

parser = argparse.ArgumentParser(description='Tool to demonstrate FUNMAP navigation.')
# TODO: support --load option
# parser.add_argument("--load", nargs="?", type=str, const="",
#                     help="Load head scan from given filepath. Use the funmap_head_scan_visualizer.py tool to create new head scans.")
parser.add_argument("--voi-side", type=float, default=8.0,
                    help="The length in meters for the map volume of interest.")
args, _ = parser.parse_known_args()

robot = stretch_pyfunmap.robot.FunmapRobot()
robot.body.stow() # Reduce occlusion from the arm and gripper

# Execute head scanning full procedure
head_scanner = ma.HeadScan(robot, voi_side_m=args.voi_side)
head_scanner.execute_full()

# User selects a desired base pose
input("""
Press enter. In the window that appears, you will see a MHI map
and the robot's pose marked with a red circle & line (orientation).
Mouse over the image to a desired base goal, and note the pixel
position in the bottom left. Multiple it by 2. Press 'q' to close
the window, and enter the pixel position in the next prompt.
""")
display_head_scan('Head Scan', head_scanner, scale_divisor=2)
base_goal_str = input("""Enter desired goal as xya tuple (e.g. "(1000, 1000, 0.0)"): """)
base_goal = eval(base_goal_str)
input("""
Press enter. In the window that appears, you will see the goal
pose marked with a green circle & line. Verify it is correct.
If correct, press 'q' to close. If it isn't, use Ctrl-C to exit.
""")
display_head_scan('Head Scan', head_scanner, scale_divisor=2, robot_xya_pix_list=[list(base_goal)])

# Plan a path
move_base = nv.MoveBase(robot)

robot.body.stop()
