#!/usr/bin/env python3

# Sample command line usage:
#   funmap_head_scan_visualizer.py --save ./example_scan
# See the parser arguments below for more command-line options.

import argparse
import stretch_pyfunmap.robot
import stretch_pyfunmap.mapping as ma
from stretch_pyfunmap.mapping import display_head_scan

parser = argparse.ArgumentParser(description='Tool to create and visualize a head scan.')
parser.add_argument("--save", nargs="?", type=str, const="",
                    help="Save head scan to given filepath at end of script.")
parser.add_argument("--voi-side", type=float, default=16.0,
                    help="The length in meters for the map volume of interest.")
args, _ = parser.parse_known_args()

robot = stretch_pyfunmap.robot.FunmapRobot()
robot.body.stow() # Reduce occlusion from the arm and gripper

# Execute head scanning full procedure
head_scanner = ma.HeadScan(robot, voi_side_m=args.voi_side)
head_scanner.execute_full()
if args.save:
    head_scanner.save(args.save)
display_head_scan('Head Scan', head_scanner)

robot.body.stop()
