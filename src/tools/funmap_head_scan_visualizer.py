import stretch_pyfunmap.robot

robot = stretch_pyfunmap.robot.FunmapRobot()
robot.show_head_cam(pointcloud=True)
# robot.head_scan()
robot.body.stop()
