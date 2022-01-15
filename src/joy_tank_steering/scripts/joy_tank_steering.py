#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

left_right_axis_ind = 3
front_back_axis_ind = 4
max_linear_weels_velocity = 1.0 # default
max_angular_weels_velocity = 1.0 # default
linear_velocity = 0.0
angular_velocity = 0.0

def callback(data: Joy):
    global linear_velocity
    global angular_velocity
    if data:
        linear_velocity = max_linear_weels_velocity * data.axes[front_back_axis_ind]
        angular_velocity = max_angular_weels_velocity * data.axes[left_right_axis_ind]
    else:
        linear_velocity = 0.0
        angular_velocity = 0.0

def talker():
    rospy.init_node("tank_steering_node")
    global max_linear_weels_velocity
    global max_angular_weels_velocity
    max_linear_weels_velocity = rospy.get_param("max_linear_vel", 1.0) # [m/s]
    max_angular_weels_velocity = rospy.get_param("max_angular_vel", 1.0) # [rad/s]
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(50) # 50Hz
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0
    while not rospy.is_shutdown():
        rospy.Subscriber("joy", Joy, callback)
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        pub.publish(msg)
        rate.sleep()
    msg.linear.x = 0
    msg.angular.z = 0
    pub.publish(msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass