#! /usr/bin/env python
#
# This file is part of the Robot Learning Lab stack
#
# Copyright (C) 2018-2019 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

from math import pi

import rospy
from geometry_msgs.msg import Pose

from rll_move_client.client import RLLDefaultMoveClient
from rll_move_client.util import orientation_from_rpy


SOURCE = {"pos": 1, "disks": 3}
TARGET = {"pos": 3, "disks": 0}
HELPER = {"pos": 2, "disks": 0}

HEIGHTS = {3: 0.048, 2: 0.028, 1: 0.006, 0: 0.006}
STACK_POS = {1: -0.2, 2: 0.0, 3: 0.2}

GRIPPER_ORIENTATION = orientation_from_rpy(0, pi, pi / 2)


def hanoi(move_client, size, source, helper, target):
    if size > 0:
        # move tower of size n - 1 to helper:
        success = hanoi(move_client, size - 1, source, target, helper)
        if not success:
            return False

        # move disk from source peg to target peg
        if source["disks"]:
            success = move_disk(move_client, source, target)
            if not success:
                rospy.logerr("error moving disk")
                return False

        # move tower of size n-1 from helper to target
        success = hanoi(move_client, size - 1, helper, source, target)
        if not success:
            return False

    return True


def move_disk(move_client, start, dest):
    pose_above = Pose()
    pose_grip = Pose()

    # pick disk up
    rospy.loginfo("start has %d disks", start["disks"])
    pose_above.position.z = 0.08
    pose_above.position.x = 0.3
    pose_above.position.y = STACK_POS[start["pos"]]
    pose_above.orientation = GRIPPER_ORIENTATION
    pose_grip.position.z = HEIGHTS[start["disks"]]
    pose_grip.position.x = 0.3
    pose_grip.position.y = STACK_POS[start["pos"]]
    pose_grip.orientation = GRIPPER_ORIENTATION

    success = move_client.pick_place(pose_above, pose_grip, True, "")
    if not success:
        rospy.logerr("picking up disk failed")
        return False

    rospy.loginfo("pick up job finished")
    start["disks"] -= 1

    # place disk
    rospy.loginfo("dest has %d disks", dest["disks"])
    pose_above.position.y = STACK_POS[dest["pos"]]
    pose_grip.position.z = HEIGHTS[dest["disks"] + 1]
    pose_grip.position.y = STACK_POS[dest["pos"]]
    success = move_client.pick_place(pose_above, pose_grip, False, "")
    if not success:
        rospy.logerr("placing disk failed")
        return False

    rospy.loginfo("place job finished")
    dest["disks"] += 1

    return True


def cleanup(move_client):
    pose_above = Pose()
    pose_grip = Pose()

    # pick tower up
    pose_above.position.z = 0.03
    pose_above.position.x = 0.3
    pose_above.position.y = STACK_POS[TARGET["pos"]]
    pose_above.orientation = GRIPPER_ORIENTATION
    pose_grip.position.z = HEIGHTS[1]
    pose_grip.position.x = 0.3
    pose_grip.position.y = STACK_POS[TARGET["pos"]]
    pose_grip.orientation = GRIPPER_ORIENTATION

    success = move_client.pick_place(pose_above, pose_grip, True, "")
    if not success:
        rospy.logerr("picking up disk failed")
        return False

    rospy.loginfo("pick up job finished")

    # place disk
    pose_above.position.y = STACK_POS[SOURCE["pos"]]
    pose_grip.position.y = STACK_POS[SOURCE["pos"]]
    success = move_client.pick_place(pose_above, pose_grip, False, "")
    if not success:
        rospy.logerr("placing disk failed")
        return False

    rospy.loginfo("place job finished")

    # move a little higher before homing
    pose_above.position.z = 0.1
    success = move_client.move_lin(pose_above)
    if not success:
        rospy.logerr("moving in final pos failed")
        return False

    rospy.loginfo("ready to home")
    return True


def move_start(move_client):
    rospy.loginfo("moving above tower at start")
    pose = Pose()
    pose.position.z = 0.1
    pose.position.x = 0.3
    pose.position.y = STACK_POS[1]
    pose.orientation = GRIPPER_ORIENTATION

    success = move_client.move_ptp(pose)
    if not success:
        rospy.logerr("moving to start pos failed")
        return False

    return True


def hanoi_full(move_client):
    success = move_start(move_client)
    if not success:
        return False

    success = hanoi(move_client, SOURCE["disks"], SOURCE, HELPER, TARGET)
    if success:
        success = cleanup(move_client)

    SOURCE["disks"] = 3
    TARGET["disks"] = 0
    HELPER["disks"] = 0

    return success


def main():
    rospy.init_node('tower_hanoi')
    client = RLLDefaultMoveClient(hanoi_full)
    client.spin()


if __name__ == '__main__':
    main()
