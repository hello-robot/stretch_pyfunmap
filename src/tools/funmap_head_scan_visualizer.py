import time
import stretch_pyfunmap.robot
import stretch_pyfunmap.mapping as ma
from stretch_pyfunmap.mapping import display_head_scan

robot = stretch_pyfunmap.robot.FunmapRobot()
robot.body.stow() # Reduce occlusion from the arm and gripper

# Execute head scanning full procedure
head_scanner = ma.HeadScan(robot, voi_side_m=16.0)
head_scanner.execute_full()
display_head_scan('Head Scan', head_scanner)

robot.body.stop()
