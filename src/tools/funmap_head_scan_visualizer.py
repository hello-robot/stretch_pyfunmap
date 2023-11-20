import time
import stretch_pyfunmap.robot
import stretch_pyfunmap.mapping as ma

robot = stretch_pyfunmap.robot.FunmapRobot()
robot.body.stow() # Reduce occlusion from the arm and gripper

# Execute head scanning full procedure
head_scanner = ma.HeadScan(robot, voi_side_m=16.0)
head_scanner.execute_full()
head_scanner.save('/home/hello-robot/repos/stretch_pyfunmap/src/tools/example_scan')

robot.body.stop()
