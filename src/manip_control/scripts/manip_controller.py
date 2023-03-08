#!/usr/bin/python3
import rospy
import math
import random
import traceback
from geometry_msgs.msg import PointStamped
from sirius_msgs.msg import ManipPose

from ik import IKSolver
from ros_interface import ManipInterface, ROSManipInterface
from motion_interpolation import InterpolationSettings
from motion_strategies import CartesianMotion, JointspaceMotion


class SiriusManip:

    def __init__(self, manip_interface: ManipInterface):
        self.manip_interface = manip_interface

        self.solver = IKSolver(self.manip_interface.get_manip_params()["links"]["names"],
                               self.manip_interface.get_manip_params()["links"]["lengths"],
                               self.manip_interface.get_manip_params()["links"]["limits"])
        rospy.Subscriber("/clicked_point", PointStamped, self.point_callback, queue_size=10)

    def point_callback(self, data):
        rospy.loginfo("Received request:\n" + str(data.point))
        target = ManipPose()
        target.x = data.point.x
        target.y = data.point.y
        target.z = data.point.z
        target.pitch = random.uniform(-math.pi / 4, 0)
        self.move_jointspace(target)

    def move_cartesian(self, target_pose: ManipPose):
        self._move(target_pose, CartesianMotion, "cartesian")

    def move_jointspace(self, target_pose: ManipPose):
        self._move(target_pose, JointspaceMotion, "jointspace")

    def _get_controlmode_settings(self, mode_name: str):
        interpolation_settings = InterpolationSettings(
            acceleration=self.manip_interface.get_manip_params()["control_modes"][mode_name]["acceleration"],
            max_velocity=self.manip_interface.get_manip_params()["control_modes"][mode_name]["max_velocity"],
            max_error=self.manip_interface.get_manip_params()["control_modes"][mode_name]["max_error"])
        rate = self.manip_interface.get_manip_params()["control_modes"][mode_name]["interpolation_rate"]
        return interpolation_settings, rate

    def _move(self, target_pose: ManipPose, motion_strategy, mode_name: str):
        interpolation_settings, rate = self._get_controlmode_settings(mode_name)
        motion = motion_strategy(target_pose, interpolation_settings, self.solver, rate)
        motion.execute(self.manip_interface)
        rospy.loginfo("Reached target point.")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("manip_controller")
    interface = ROSManipInterface()
    SiriusManip(interface).run()
