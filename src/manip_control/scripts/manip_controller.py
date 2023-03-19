#!/usr/bin/python3
import rospy
import math
import random
import traceback
from geometry_msgs.msg import PointStamped

from ik import SiriusII_IKSolver, ManipPose, ManipJointState
from manip_interface import ManipInterface
from manip_interface_ros import ROSManipInterface, PosePublishingDecorator, JointstatePublishingDecorator
from manip_interface_dummy import DummyManipInterface
from motion_interpolation import InterpolationSettings
from motion_strategies import CartesianMotion, JointspaceMotion


class Manip:

    def __init__(self, manip_interface: ManipInterface):
        self.manip_interface = manip_interface
        self.params = self.manip_interface.get_manip_params()
        self.solver = self._create_ik_solver(self.params)

    def _create_ik_solver(self, params):
        return SiriusII_IKSolver(params.joint_names(), params.link_lengths(), params.joint_limits())

    def move_cartesian(self, target_pose: ManipPose):
        self._move(target_pose, CartesianMotion, "cartesian")

    def move_jointspace(self, target_pose: ManipPose):
        self._move(target_pose, JointspaceMotion, "jointspace")

    def _get_controlmode_settings(self, mode_name: str):
        mode_params = self.params.control_mode_params(mode_name)
        interpolation_settings = InterpolationSettings.from_params(mode_params)
        rate = mode_params["interpolation_rate"]
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
        interface = DummyManipInterface()
        interface.set_manip_params(ROSManipInterface().get_manip_params())
        interface.set_jointstate(ManipJointState.from_list([0, 0, 0, 0, 0, 0]))
        interface.set_time_scale(1)
        interface = JointstatePublishingDecorator(interface)
        interface.set_topic_name("/joint_states")
        interface = PosePublishingDecorator(interface)
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
