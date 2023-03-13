#!/usr/bin/python3
import rospy
import math
import random
import traceback
from geometry_msgs.msg import PointStamped

from ik import IKSolver, ManipPose
from ros_interface import ManipInterface, ROSManipInterface, PosePublishingDecorator
from motion_interpolation import InterpolationSettings
from motion_strategies import CartesianMotion, JointspaceMotion


class Manip:

    def __init__(self, manip_interface: ManipInterface):
        self.manip_interface = manip_interface
        self.solver = self._create_ik_solver()

    def _create_ik_solver(self):
        return IKSolver(self.manip_interface.get_manip_params()["links"]["names"],
                        self.manip_interface.get_manip_params()["links"]["lengths"],
                        self.manip_interface.get_manip_params()["links"]["limits"])

    def move_cartesian(self, target_pose: ManipPose):
        self._move(target_pose, CartesianMotion, "cartesian")

    def move_jointspace(self, target_pose: ManipPose):
        self._move(target_pose, JointspaceMotion, "jointspace")

    def _get_controlmode_settings(self, mode_name: str):
        params = self.manip_interface.get_manip_params()["control_modes"][mode_name]
        interpolation_settings = InterpolationSettings.from_params(params)
        rate = self.manip_interface.get_manip_params()["control_modes"][mode_name]["interpolation_rate"]
        return interpolation_settings, rate

    def _move(self, target_pose: ManipPose, motion_strategy, mode_name: str):
        interpolation_settings, rate = self._get_controlmode_settings(mode_name)
        motion = motion_strategy(target_pose, interpolation_settings, self.solver, rate)
        motion.execute(self.manip_interface)
        rospy.loginfo("Reached target point.")

    def get_ik_solver(self):
        return self.solver


class ManipNode:

    def __init__(self):
        rospy.init_node("manip_controller")
        interface = PosePublishingDecorator(ROSManipInterface())
        self.manip = Manip(interface)
        interface.set_ik_solver(self.manip.get_ik_solver())
        interface.set_topic_name("/received_point")
        rospy.Subscriber("/clicked_point", PointStamped, self.callback, queue_size=10)

    def run(self):
        rospy.spin()

    def callback(self, data):
        rospy.loginfo("Received request:\n" + str(data.point))
        target = ManipPose()
        target.x = data.point.x
        target.y = data.point.y
        target.z = data.point.z
        target.pitch = random.uniform(-math.pi / 4, 0)
        target.roll = random.uniform(-math.pi, math.pi)
        self.manip.move_cartesian(target)


if __name__ == "__main__":
    ManipNode().run()
