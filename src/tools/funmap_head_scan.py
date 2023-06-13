import time
import pathlib
import ros_numpy
import numpy as np
import stretch_body.hello_utils as hu
from urdfpy import URDF
from tiny_tf.tf import TFTree, Transform


class FakeRobot:

    def __init__(self):
        urdf_path = str((pathlib.Path(hu.get_fleet_directory()) / 'exported_urdf' / 'stretch.urdf').absolute())
        self.urdf = URDF.load(urdf_path)
        # starting in 'stow' pose
        self.joint_state = {
            'joint_left_wheel': 0.0,
            'joint_right_wheel': 0.0,
            'joint_lift': 0.2,
            'joint_arm_l0': 0.0,
            'joint_arm_l1': 0.0,
            'joint_arm_l2': 0.0,
            'joint_arm_l3': 0.0,
            'joint_wrist_yaw': 3.4,
            'joint_wrist_pitch': 0.0,
            'joint_wrist_roll': 0.0,
            'joint_gripper_finger_left': 0.0,
            'joint_gripper_finger_right': 0.0,
            'joint_head_pan': 0.0,
            'joint_head_tilt': 0.0
        }

    def move_to_pose(self, pose, return_before_done=False, custom_contact_threshold=False):
        if custom_contact_threshold:
            raise Exception('FakeRobot.move_to_pose: custom contact threshold not supported')
        if 'joint_gripper_finger_left' in pose or 'joint_gripper_finger_right' in pose:
            raise Exception('FakeRobot.move_to_pose: gripper cmd not supported')

        for joint in pose:
            self.joint_state[joint] = pose[joint]

        if 'wrist_extension' in self.joint_state:
            # update individual arm links
            for i in range(4):
                self.joint_state[f'joint_arm_l{i}'] = self.joint_state['wrist_extension'] / 4.0
            self.joint_state.pop('wrist_extension')

    def lookup_transform(self, target_frame, source_frame):
        # update self.tf2_buffer
        tree = TFTree()
        fk_curr = self.urdf.link_fk(cfg=self.joint_state)
        for l in self.urdf.links:
            if l.name == "base_link":
                continue
            tree.add_transform('base_link', l.name, Transform.from_matrix(fk_curr[l]))
        tree.add_transform('map', 'base_link', Transform())

        # return tf
        return tree.lookup_transform(target_frame, source_frame)

    def get_point_cloud(self):
        npoints = 100
        points_arr = np.zeros((npoints,), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('r', np.uint8),
            ('g', np.uint8),
            ('b', np.uint8)])
        points_arr['x'] = np.random.random((npoints,))
        points_arr['y'] = np.random.random((npoints,))
        points_arr['z'] = np.random.random((npoints,))
        points_arr['r'] = np.floor(np.random.random((npoints,))*255)
        points_arr['g'] = 0
        points_arr['b'] = 255
        points_arr = ros_numpy.point_cloud2.merge_rgb_fields(points_arr)

        cloud_time = time.time()
        cloud_frame_id = 'camera_link'
        cloud_arr = points_arr
        return (cloud_time, cloud_frame_id, cloud_arr)

# -----------------------------------------------------------

import stretch_funmap.mapping as ma

node = FakeRobot()
scanner = ma.HeadScan(voi_side_m=16.0)
# scanner.execute_full(node, fast_scan=False)
# scanner.save('/home/binitshah/repos/stretch_funmap/example_head_scan')
