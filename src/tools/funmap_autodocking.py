#!/usr/bin/env python3

import sys
import time
import stretch_body.robot
import stretch_pyfunmap.robot
import stretch_pyfunmap.mapping as ma
from stretch_body.gamepad_teleop import GamePadTeleop

# 1. See if already connected to a charger
body = stretch_body.robot.Robot()
body.startup()
if body.pimu.status['charger_connected']:
    print("Already on charger")
    # sys.exit(1)

# 2. Initialize Funmap, Aruco detection, and gamepad teleop
robot = stretch_pyfunmap.robot.FunmapRobot(body=body)
gamepad_teleop = GamePadTeleop(robot_instance=False)
gamepad_teleop.startup(robot.body)
found_dock = False
def step3_complete():
    global found_dock
    found_dock = True
gamepad_teleop.left_stick_button_fn = step3_complete

# 3. Teleop the robot to a random location within 3m of the dock
#    and point the head cam at dock's aruco marker
imwrite_counter = 0
while not found_dock:
    imwrite_counter = (imwrite_counter + 1) % 100
    if imwrite_counter == 99:
        # print("Writing Image")
        robot.detect_arucos(imwrite='/home/hello-robot/repos/stretch_pyfunmap/src/tools/view.png')
    # print(f"Charging={robot.body.pimu.status['charger_connected']}")
    gamepad_teleop.step_mainloop(robot.body)

# 4. Create a head scan of the area around the docking station
head_scanner = ma.HeadScan(robot, voi_side_m=8.0)
head_scanner.execute_front(fast_scan=True)
print(len(head_scanner.unprocessed_data))
from stretch_pyfunmap.mapping import display_head_scan
display_head_scan('Head Scan', head_scanner)

gamepad_teleop.stop()
robot.body.stop()
