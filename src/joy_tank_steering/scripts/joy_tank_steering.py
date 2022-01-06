#!/usr/bin python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

left_right_axis_ind = 2
front_back_axis_ind = 3
max_linear_weels_velocity = 1 # to add
max_angular_weels_velocity = 1 # to add
linear_velocity = 0
angular_velocity = 0

def callback(data):
    linear_velocity = max_linear_weels_velocity * data.axes[front_back_axis_ind]
    angular_velocity = max_angular_weels_velocity * data.axes[left_right_axis_ind]

def talker():
    rospy.init_node("tank_steering_node")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(50) # 50Hz
    while not rospy.is_shutdown():
        rospy.Subscriber("joy", Joy, callback)
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.linear.y = 0
        msg.linear.x = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = angular_velocity
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass