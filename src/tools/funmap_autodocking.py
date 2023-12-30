import sys
import time
import stretch_pyfunmap.robot
from stretch_body.gamepad_teleop import GamePadTeleop

# 1. See if already connected to a charger
robot = stretch_pyfunmap.robot.FunmapRobot(autoexposure_timeout=0.0)
if robot.body.pimu.status['charger_connected']:
    print("Already on charger")

# # print robot joint positions
# print(f"Head Pan Position = {robot.body.head.status['head_pan']['pos']}rad")
# print(f"Head Tilt Position = {robot.body.head.status['head_tilt']['pos']}rad")
# print(f"Lift Position = {robot.body.lift.status['pos']}m")
# print(f"Wrist Yaw Position = {robot.body.end_of_arm.status['wrist_yaw']['pos']}")

# 2. Prepare robot
start = time.time()
while (time.time() - start < 2.0):
    robot.head_cam.wait_for_frames()
robot.body.stow() # Reduce occlusion from the arm and gripper

imwrite_counter = 0
gamepad_teleop = GamePadTeleop(robot_instance=False)
gamepad_teleop.startup(robot.body)
while True:
    imwrite_counter = (imwrite_counter + 1) % 100
    if imwrite_counter == 99:
        print("Writing Image")
        robot.show_head_cam(imwrite='/home/hello-robot/repos/stretch_pyfunmap/src/tools/view.png')
    print(f"Charging={robot.body.pimu.status['charger_connected']}")
    gamepad_teleop.step_mainloop(robot.body)

gamepad_teleop.stop()
robot.body.stop()
