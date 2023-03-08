#!/usr/bin/python3
import rospy
import math
import random
import traceback
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from sirius_msgs.msg import ManipPose
from std_msgs.msg import Float64

from ik import IKSolver
from ros_interface import ManipInterface, ROSManipInterface
from motion_interpolation import InterpolationSettings
from motion_strategies import CartesianMotion, JointspaceMotion


#TODO: decouple logic and ROS specific code.
class SiriusManip:

    def __init__(self, manip_interface: ManipInterface):
        self.manip_interface = manip_interface

        self.MODES_DATA = rospy.get_param("~control_modes")
        self.LINKS_DATA = rospy.get_param("~links")
        self.solver = IKSolver(self.LINKS_DATA["names"], self.LINKS_DATA["lengths"], self.LINKS_DATA["limits"])

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
        interpolation_settings = InterpolationSettings(
            acceleration=self.MODES_DATA["cartesian"]["acceleration"],
            max_velocity=self.MODES_DATA["cartesian"]["max_velocity"],
            max_error=self.MODES_DATA["cartesian"]["max_error"]
        )
        rate = self.MODES_DATA["cartesian"]["interpolation_rate"]
        motion = CartesianMotion(target_pose, interpolation_settings, self.solver, rate)
        motion.execute(self.manip_interface)

        rospy.loginfo("Reached target point.")

    def move_jointspace(self, target_pose: ManipPose):
        interpolation_settings = InterpolationSettings(
            acceleration=self.MODES_DATA["jointspace"]["acceleration"],
            max_velocity=self.MODES_DATA["jointspace"]["max_velocity"],
            max_error=self.MODES_DATA["jointspace"]["max_error"]
        )
        rate = self.MODES_DATA["jointspace"]["interpolation_rate"]
        motion = JointspaceMotion(target_pose, interpolation_settings, self.solver, rate)
        motion.execute(self.manip_interface)

        rospy.loginfo("Reached target point.")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("manip_controller")
    interface = ROSManipInterface()
    SiriusManip(interface).run()
