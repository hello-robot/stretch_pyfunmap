#!/usr/bin/env python3

import stretch_pyfunmap.navigate as nv

from actionlib_msgs.msg import GoalStatus


class ROSMoveBase(nv.MoveBase):

    def __init__(self, node, debug_directory=None):
        super().__init__(node, debug_directory)
        self.node = self.robot
        self.unsuccessful_status = [GoalStatus.PREEMPTED, GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.LOST]

    def check_move_state(self, trajectory_client):
        at_goal = False
        unsuccessful_action = False
        state = trajectory_client.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo('Move succeeded!')
            # wait for the motion to come to a complete stop
            rospy.sleep(0.5)
            at_goal = True
        elif state in self.unsuccessful_status:
            rospy.loginfo('Move action terminated without success (state = {0}).'.format(state))
            unsuccessful_action = True
        return at_goal, unsuccessful_action
