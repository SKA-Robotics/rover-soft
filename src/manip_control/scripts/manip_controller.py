#!/usr/bin/python3
import rospy
import math
import random
import traceback
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from sirius_msgs.msg import ManipPose
from std_msgs.msg import Float64

from sirius_ik import IKSolver
from ros_interface import ManipInterface, ROSManipInterface
from motion_planning import MotionInterpolator


#TODO: decouple logic and ROS specific code.
class SiriusManip:

    def __init__(self, manip_interface : ManipInterface):
        self.manip_interface = manip_interface

        self.MODES_DATA = rospy.get_param("~control_modes")
        self.LINKS_DATA = rospy.get_param("~links")
        self.solver = IKSolver(self.LINKS_DATA["names"], self.LINKS_DATA["lengths"], self.LINKS_DATA["limits"])

        rospy.Subscriber("/clicked_point", PointStamped, self.point_callback, queue_size=10)
        self.point_publisher = rospy.Publisher("/received_point", PointStamped, queue_size=10)

    def point_callback(self, data):
        rospy.loginfo("Received request:\n" + str(data.point))
        target = ManipPose()
        target.x = data.point.x
        target.y = data.point.y
        target.z = data.point.z
        target.pitch = random.uniform(-math.pi / 4, 0)

        self.move_jointspace(target)

    def _move(self, pose: ManipPose):
        jointstate = JointState()
        try:
            jointstate = self.solver.get_IK_solution(pose)
            self._publish_jointstate(jointstate)
        except Exception as e:
            #TODO add diagnostic information
            rospy.logwarn("Could not move to target point")
            rospy.logwarn(traceback.format_exc())
            return

        ik_fk = self.solver.get_FK_solution(jointstate)
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "root_link"
        msg.point.x = ik_fk.x
        msg.point.y = ik_fk.y
        msg.point.z = ik_fk.z
        self.point_publisher.publish(msg)

    def _publish_jointstate(self, jointstate: JointState):
        self.manip_interface.set_jointstate(jointstate)

    def get_pose(self) -> ManipPose:
        return self.solver.get_FK_solution(self.get_jointstate())

    def get_jointstate(self) -> JointState:
        return self.manip_interface.get_jointstate()

    # For traversing small distances in a straight line. Using this
    # mode to span large distances may lead to errors as the manipulator
    # would enter collision with itself
    def move_cartesian(self, target_pose: ManipPose):
        pose = self.get_pose()
        # Load movement parameters
        acceleration = self.MODES_DATA["cartesian"]["acceleration"]
        max_velocity = self.MODES_DATA["cartesian"]["max_velocity"]
        error = self.MODES_DATA["cartesian"]["max_error"]
        rate = rospy.Rate(self.MODES_DATA["cartesian"]["interpolation_rate"])

        interpolator = MotionInterpolator(acceleration=acceleration, max_velocity=max_velocity, max_error=error)
        interpolator.set_movement([pose.x, pose.y, pose.z, pose.pitch], [target_pose.x, target_pose.y, target_pose.z, target_pose.pitch])
        
        while interpolator.is_not_done():
            pose_list = interpolator.movement_step(rate.sleep_dur.to_sec())
            pose.x = pose_list[0]
            pose.y = pose_list[1]
            pose.z = pose_list[2]
            pose.pitch = pose_list[3]
            self._move(pose)
            rate.sleep()

        rospy.loginfo("Reached target point.")

    # For executing long movements. No chance of big trouble
    def move_jointspace(self, target_pose: ManipPose):
        jointstate = self.solver.get_IK_solution(self.get_pose())
        target_jointstate = self.solver.get_IK_solution(target_pose)

        # Load movement parameters
        acceleration = self.MODES_DATA["jointspace"]["acceleration"]
        max_velocity = self.MODES_DATA["jointspace"]["max_velocity"]
        error = self.MODES_DATA["jointspace"]["max_error"]
        rate = rospy.Rate(self.MODES_DATA["jointspace"]["interpolation_rate"])

        interpolator = MotionInterpolator(acceleration=acceleration, max_velocity=max_velocity, max_error=error)
        interpolator.set_movement(jointstate.position, target_jointstate.position)

        while interpolator.is_not_done():
            positions = interpolator.movement_step(rate.sleep_dur.to_sec())
            jointstate.position = positions
            self._publish_jointstate(jointstate)
            rate.sleep()

        rospy.loginfo("Reached target point.")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("manip_controller")
    interface = ROSManipInterface()
    SiriusManip(interface).run()
