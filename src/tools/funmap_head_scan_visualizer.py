import stretch_pyfunmap.robot

robot = stretch_pyfunmap.robot.FunmapRobot()
robot.show_head_cam(apply_global_histogram_equalization=True)
# robot.head_scan()
robot.body.stop()
