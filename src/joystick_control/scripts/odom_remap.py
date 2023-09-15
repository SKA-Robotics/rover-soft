#!/usr/bin/python3
import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

class OdomRemap:
    def __init__(self) -> None:
        rospy.init_node("odom_remap")

        self.odom = rospy.Publisher("/odom", Odometry, queue_size=10)

        self.wheels_left = rospy.Subscriber("/wheels_left/motor0/joint_state", JointState, self.wheels_left_call)
        self.wheels_right = rospy.Subscriber("/wheels_right/motor0/joint_state", JointState, self.wheels_right_call)

        self.vel_left = 0.0
        self.vel_right = 0.0


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg = Odometry()

            r = 0.1
            L = 0.69 * 2
            msg.twist.twist.linear.x = r / 2 * (self.vel_left + self.vel_right)
            msg.twist.twist.angular.z = r / L * (self.vel_right - self.vel_left)

            msg.twist.covariance[0] = 0.2
            msg.twist.covariance[7] = 0.05
            msg.twist.covariance[14] = 0.05
            msg.twist.covariance[35] = 1.0

            self.odom.publish(msg)
            rate.sleep()

    def wheels_left_call(self, joint_state):
        self.vel_left = joint_state.velocity[0]

    def wheels_right_call(self, joint_state):
        self.vel_right = joint_state.velocity[0]



if __name__ == "__main__":
    try:
        OdomRemap().run()
    except rospy.ROSInterruptException:
        pass
