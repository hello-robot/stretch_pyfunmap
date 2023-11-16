import stretch_pyfunmap.robot

robot = stretch_pyfunmap.robot.FunmapRobot()
if robot.body.is_calibrated():
    print('Stretch is ready to go')
robot.body.stop()
