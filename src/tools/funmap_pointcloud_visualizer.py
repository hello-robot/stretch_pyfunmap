import stretch_pyfunmap.robot

robot = stretch_pyfunmap.robot.FunmapRobot()
robot.body.stow() # Reduce occlusion from the arm and gripper

robot.show_head_cam(hold=False, align=False, pointcloud=True)

robot.body.stop()
