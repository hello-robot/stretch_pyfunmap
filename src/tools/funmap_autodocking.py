import stretch_pyfunmap.robot

robot = stretch_pyfunmap.robot.FunmapRobot()
robot.body.stow() # Reduce occlusion from the arm and gripper

robot.body.stop()
