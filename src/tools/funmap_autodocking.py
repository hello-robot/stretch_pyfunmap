#!/usr/bin/env python3

import sys
import time
import stretch_body.robot
import stretch_pyfunmap.robot
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

# 3. Teleop the robot to a random location within 3m of the dock
imwrite_counter = 0
while True:
    imwrite_counter = (imwrite_counter + 1) % 100
    if imwrite_counter == 99:
        print("Writing Image")
        robot.show_head_cam(imwrite='/home/hello-robot/repos/stretch_pyfunmap/src/tools/view.png')
    # print(f"Charging={robot.body.pimu.status['charger_connected']}")
    gamepad_teleop.step_mainloop(robot.body)

gamepad_teleop.stop()
robot.body.stop()
