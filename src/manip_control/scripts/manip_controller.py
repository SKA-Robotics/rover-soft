#!/usr/bin/python3
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped

from sirius_ik import IKSolver, IKTarget

class SiriusManip:
    def __init__(self):
        rospy.init_node("manip_controller")

        # TODO read linkLengths and limits of actual manip
        linkLengths = [0.0655, 0.43498, 0.5236, 0.129]
        limits = [(-3, 3), (-3, 3), (-3, 3), (-3, 3)]
        self.solver = IKSolver(linkLengths, limits)

        rospy.Subscriber("/clicked_point", PointStamped, self.point_callback, queue_size=10)
        self.point_publisher = rospy.Publisher("/received_point", PointStamped, queue_size=10)
        self.jointstate_publisher = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.rate = rospy.Rate(10.0)

    def point_callback(self, data):
        rospy.loginfo("Received request:\n" + str(data.point))
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.point = data.point
        self.point_publisher.publish(msg)

        self.moveTo(point=data, pitch=0.0)

    def moveTo(self, point : PointStamped, pitch : float):
        target = IKTarget(point.point.x, point.point.y, point.point.z, pitch)
        try:
            jointstate = self.solver.getIKSolution(target)
            jointstate.header.stamp = rospy.Time.now()
            jointstate.name = [ "base_cyl",
                                "cyl_arm1",
                                "arm1_arm2",
                                "arm2_arm3"
                              ]
            self.jointstate_publisher.publish(jointstate)
        except Exception as e:
            #TODO add diagnostic information
            rospy.logwarn("Could not move to target point")
            rospy.logwarn(e)

    def run(self):
        rospy.spin()


if __name__=="__main__":
    SiriusManip().run()