import sys
import stretch_body.robot


class FunmapRobot:

    def __init__(self, body=None):
        self.body = body
        if self.body is None:
            self.body = stretch_body.robot.Robot()
            did_start = self.body.startup()
            if not did_start:
                print('CRITICAL: hardware failed to initialize completely')
                self.body.stop()
                sys.exit(1)
        if not self.body.is_calibrated():
            self.body.home()
