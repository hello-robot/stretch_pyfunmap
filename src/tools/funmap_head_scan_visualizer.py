import stretch_pyfunmap.robot

robot = stretch_pyfunmap.robot.FunmapRobot()
# robot.show_head_cam(pointcloud=True)
scan = robot.head_scan()
scan.save('/home/hello-robot/repos/stretch_pyfunmap/src/tools/example_scan')
robot.body.stop()
