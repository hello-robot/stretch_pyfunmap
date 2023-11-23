import pyrealsense2 as rs
import stretch_pyfunmap.robot

# Get all camera infos
camera_info = [{
    'name': device.get_info(rs.camera_info.name),
    'serial_number': device.get_info(rs.camera_info.serial_number)
} for device in rs.context().devices]

# Filter out D405
d405_info = None
for info in camera_info:
    if info['name'].endswith('D405'):
        d405_info = info
if d405_info is None:
    print('funmap_stub_test.py FAIL: couldnt find stub camera')

# initialize camera
pipeline = rs.pipeline()
fps = 15
resolution = (640, 480) # (1280, 720) @ 5fps, (848, 480) @ 10fps, (640, 480) @ 30 fps
config = rs.config()
config.enable_device(d405_info['serial_number'])
config.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, fps)
config.enable_stream(rs.stream.depth, resolution[0], resolution[1], rs.format.z16, fps)
profile = pipeline.start(config)

# initialize stubbed body
class StubBody():
    def __init__(self):
        self._is_calibrated = True

    def is_calibrated(self):
        return self._is_calibrated

    def stop(self):
        pass
body = StubBody()

# initialize funmap robot
robot = stretch_pyfunmap.robot.FunmapRobot(body=body, head_cam=pipeline, enable_networking=False)
robot.show_head_cam()
