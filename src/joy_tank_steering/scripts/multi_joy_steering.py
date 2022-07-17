#!/usr/bin/python3
from enum import Enum
import rospy
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist

# axes numbers
LEFT_STICK_HORIZONTAL = 0
LEFT_STICK_VERTICAL = 1
RIGHT_STICK_HORIZONTAL = 3
RIGHT_STICK_VERTICAL = 4
LEFT_TRIGGER = 5
RIGHT_TRIGGER = 2

# buttons numbers
LEFT_BUMPER = 4
RIGHT_BUMPER = 5
BACK_BUTTON = 6
START_BUTTON = 7


class JoystickController:

    def __init__(self):
        self.id = rospy.get_param("~ID", "0")
        self.name = "joystick_steering_node_" + self.id
        rospy.init_node(self.name)
        self.joy_topic_name = rospy.get_param("~joy_message_topic_name", "joy")
        self.tank_steering_mode: bool = rospy.get_param("~tank_steering_mode", "True")
        self.wheel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.manipulator_pub = rospy.Publisher("cmd_manip", JointState, queue_size=10)
        rospy.Subscriber(self.joy_topic_name, Joy, self.joy_steer_callback)
        # rover steering as default
        if rospy.get_param("~start_steering_mode", "0") == 0:
            self.steering_mode = self.SteeringMode.ROVER_STEERING
        else:
            self.steering_mode = self.SteeringMode.MANIPULATOR_STEERING
        # all velocities are scaled in range (-1,1) corresponding to a percentage of maximal speed in both sides
        self.manipulator_limbs_vel = {
            'arm_rotate': 0,
            'arm_lift': 0,
            'claw_rotate': 0,
            'claw_lift': 0,
            'arm_tilt': 0,
            'claw_clamp': 0,
        }

    def __del__(self):
        # stopping rover or manipulator before killing the node
        if self.steering_mode == self.SteeringMode.ROVER_STEERING:
            wheel_msg = Twist()
            self.wheel_pub.publish(wheel_msg)
        elif self.steering_mode == self.SteeringMode.MANIPULATOR_STEERING:
            manipulator_msg = JointState()
            manipulator_msg.header.stamp = rospy.get_rostime()
            manipulator_msg.name = list(self.manipulator_limbs_vel.keys())
            manipulator_msg.effort = [0] * 6
            self.manipulator_pub.publish(manipulator_msg)

    def run(self):
        rospy.spin()

    def joy_steer_callback(self, data: Joy):
        if data.buttons[START_BUTTON] and not data.buttons[BACK_BUTTON]:
            self.steering_mode = self.SteeringMode.ROVER_STEERING
        elif data.buttons[BACK_BUTTON] and not data.buttons[START_BUTTON]:
            self.steering_mode = self.SteeringMode.MANIPULATOR_STEERING

        if self.steering_mode == self.SteeringMode.ROVER_STEERING:
            wheel_msg = Twist()
            if self.tank_steering_mode:
                wheel_msg.linear.x = (data.axes[LEFT_STICK_VERTICAL] + data.axes[RIGHT_STICK_VERTICAL]) / 2
                wheel_msg.angular.z = (data.axes[RIGHT_STICK_VERTICAL] - data.axes[LEFT_STICK_VERTICAL]) / 2
            else:
                wheel_msg.linear.x = data.axes[RIGHT_STICK_VERTICAL]
                wheel_msg.angular.z = data.axes[RIGHT_STICK_HORIZONTAL]
            self.wheel_pub.publish(wheel_msg)

        elif self.steering_mode == self.SteeringMode.MANIPULATOR_STEERING:
            self.manipulator_limbs_vel['arm_rotate'] = -data.axes[LEFT_STICK_HORIZONTAL]
            self.manipulator_limbs_vel['arm_lift'] = -data.axes[LEFT_STICK_VERTICAL]
            self.manipulator_limbs_vel['claw_rotate'] = data.axes[RIGHT_STICK_HORIZONTAL]
            self.manipulator_limbs_vel['claw_lift'] = -data.axes[RIGHT_STICK_VERTICAL]
            self.manipulator_limbs_vel['arm_tilt'] = -(data.axes[LEFT_TRIGGER] - data.axes[RIGHT_TRIGGER]) / 2
            self.manipulator_limbs_vel['claw_clamp'] = data.buttons[LEFT_BUMPER] - data.buttons[RIGHT_BUMPER]
            manipulator_msg = JointState()
            manipulator_msg.header.stamp = rospy.get_rostime()
            manipulator_msg.name = list(self.manipulator_limbs_vel.keys())
            manipulator_msg.effort = list(self.manipulator_limbs_vel.values())
            self.manipulator_pub.publish(manipulator_msg)

    class SteeringMode(Enum):
        ROVER_STEERING = 0
        MANIPULATOR_STEERING = 1


if __name__ == '__main__':
    try:
        JoystickController().run()
    except rospy.ROSInterruptException:
        pass
