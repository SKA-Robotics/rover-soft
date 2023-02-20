#!/usr/bin/python3
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from sirius_msgs.msg import ManipPose

from sirius_ik import IKSolver


# Class for controlling the manipulator
class SiriusManip:

    def __init__(self):
        rospy.init_node("manip_controller")

        # TODO read linkLengths and limits of actual manip
        linkLengths = [0.0655, 0.43498, 0.5236, 0.129]
        limits = [(-3, 3), (-3, 3), (-3, 3), (-3, 3)]
        self.solver = IKSolver(linkLengths, limits)

        rospy.Subscriber("/clicked_point",
                         PointStamped,
                         self.point_callback,
                         queue_size=10)
        self.point_publisher = rospy.Publisher("/received_point",
                                               PointStamped,
                                               queue_size=10)
        self.jointstate_publisher = rospy.Publisher("/joint_states",
                                                    JointState,
                                                    queue_size=10)
        self.rate = rospy.Rate(10.0)

    def point_callback(self, data):
        rospy.loginfo("Received request:\n" + str(data.point))

        target = ManipPose()
        target.x = data.point.x
        target.y = data.point.y
        target.z = data.point.z
        target.pitch = -3.14 / 6

        self.moveTo(target)

    def moveTo(self, pose: ManipPose):
        jointstate = JointState()
        try:
            jointstate = self.solver.get_IK_solution(pose)
            jointstate.header.stamp = rospy.Time.now()
            self.jointstate_publisher.publish(jointstate)
        except Exception as e:
            #TODO add diagnostic information
            rospy.logwarn("Could not move to target point")
            rospy.logwarn(e)

        ik_fk = self.solver.get_FK_solution(jointstate)
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "root_link"
        msg.point.x = ik_fk.x
        msg.point.y = ik_fk.y
        msg.point.z = ik_fk.z
        self.point_publisher.publish(msg)

    # TODO add straight-line movement (target interpolation)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    SiriusManip().run()
