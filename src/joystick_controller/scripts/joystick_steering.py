#!/usr/bin/python3
from enum import Enum
from math import tan
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# used topics names
ROVER_TOPIC_NAME = "/cmd_vel"
MANIP_TOPIC_NAME = "/cmd_manip"
GRIPPER_TOPIC_NAME = "/cmd_grip"


class JoystickController:

    def __init__(self):
        ID = rospy.get_param("~ID", "0")
        NAME = "joystick_steering_node_" + ID
        rospy.init_node(NAME)
        JOY_TOPIC_NAME = rospy.get_param("~joy_message_topic_name", "joy")

        # set rover steering and normal modes at begining
        self.steered_object = self.SteeredObject.ROVER
        self.rover_steering_mode = self.RoverSteeringMode.NORMAL
        self.manip_steering_mode = self.ManipSteeringMode.INVERSE_KINEMATICS

        # get axes and buttons configuration from the file
        JOYSTICK_TYPE = rospy.get_param("~joystick_type", "XBOX_XS")
        JOYSTICK_DATA = rospy.get_param(f"~{JOYSTICK_TYPE}", None)
        self.MODES_DATA = rospy.get_param("~steering_modes", None)
        self.AXES_ID = JOYSTICK_DATA['axes']
        self.BUTTONS_ID = JOYSTICK_DATA['buttons']

        self.rover_publisher = rospy.Publisher(ROVER_TOPIC_NAME, Twist, queue_size=10)
        self.manip_publisher = rospy.Publisher(MANIP_TOPIC_NAME, Twist, queue_size=10)
        self.gripper_publisher = rospy.Publisher(GRIPPER_TOPIC_NAME, Float64, queue_size=10)
        rospy.Subscriber(JOY_TOPIC_NAME, Joy, self.joy_listener_callback)

        # used in child mode
        self.child_mode = rospy.get_param("~child_mode", False)
        if self.child_mode:
            i1 = min(1.0, max(rospy.get_param("~child_mode_inertia_i1", 0.4444), 0.0))
            i2 = min(1.0, max(rospy.get_param("~child_mode_inertia_i2", 0.0066), 0.0))
            self.lin_inertia = self.InertiaModel(i1, i2)
            self.ang_inertia = self.InertiaModel(i1, i2)
            rospy.loginfo("Child mode enabled")
            self.prev_rover_message = Twist()

    def __del__(self):
        # stopping rover or manipulator before killing the node
        self.stop_object()

    def run(self):
        rospy.spin()

    def joy_listener_callback(self, data: Joy):
        # convert data to much easier use
        VALUES = dict((name, data.buttons[id]) for name, id in self.BUTTONS_ID.items())
        VALUES.update(dict((name, data.axes[id]) for name, id in self.AXES_ID.items()))

        # change currently steered object and stop the previous one
        if VALUES['start_button'] and not VALUES['back_button']:
            self.stop_object()
            self.steered_object = self.SteeredObject.ROVER
            return
        if VALUES['back_button'] and not VALUES['start_button']:
            self.stop_object()
            self.steered_object = self.SteeredObject.MANIP
            return

        # change steering mode and stop object to avoid unexpected behaviour
        if self.change_mode(VALUES):
            return

        # all values are scaled in range (-1,1) corresponding to a percentage of maximal speed in both sides
        if self.steered_object == self.SteeredObject.ROVER:
            self.steer_rover(VALUES)
        elif self.steered_object == self.SteeredObject.MANIP:
            self.steer_manip(VALUES)

    def steer_rover(self, values: dict):
        rover_message = Twist()
        PARAMS = self.MODES_DATA['rover'][self.rover_steering_mode.name.lower()]
        lin_cmd = 0
        ang_cmd = 0

        if self.rover_steering_mode == self.RoverSteeringMode.NORMAL:
            lin_cmd = nonlin_scale(values['right_stick_vertical'], PARAMS['linear'])
            ang_cmd = nonlin_scale(values['right_stick_horizontal'], PARAMS['angular'])

        elif self.rover_steering_mode == self.RoverSteeringMode.TANK:
            lin_cmd = nonlin_scale((values['left_stick_vertical'] + values['right_stick_vertical']) / 2, PARAMS)
            ang_cmd = nonlin_scale((values['right_stick_vertical'] - values['left_stick_vertical']) / 2, PARAMS)

        elif self.rover_steering_mode == self.RoverSteeringMode.GAMER:
            lin_cmd = nonlin_scale((values['right_trigger'] - values['left_trigger']) / 2, PARAMS['linear'])
            turning_angle = nonlin_scale(values['right_stick_horizontal'], PARAMS['angular'])
            ang_cmd = lin_cmd * tan(turning_angle)

        if (self.child_mode):
            rover_message.linear.x = self.lin_inertia.step(lin_cmd)
            rover_message.angular.z = self.ang_inertia.step(ang_cmd)
        else:
            rover_message.linear.x = lin_cmd
            rover_message.angular.z = ang_cmd

        self.rover_publisher.publish(rover_message)

    def steer_manip(self, values: dict):
        if self.manip_steering_mode == self.ManipSteeringMode.INVERSE_KINEMATICS:
            manip_message = Twist()
            PARAMS = self.MODES_DATA['manip'][self.manip_steering_mode.name.lower()]
            manip_message.linear.x = nonlin_scale(values['cross_horizontal'], PARAMS['x'])
            manip_message.linear.y = nonlin_scale(values['cross_vertical'], PARAMS['y'])
            manip_message.linear.z = nonlin_scale(values['right_stick_vertical'], PARAMS['z'])
            manip_message.angular.x = nonlin_scale(values['left_stick_horizontal'], PARAMS['roll'])
            manip_message.angular.y = nonlin_scale(values['left_stick_vertical'], PARAMS['pitch'])
            clamp = nonlin_scale(values['left_trigger'] - values['right_trigger'], PARAMS['gripper'])
            gripper_message = Float64(clamp)

            self.manip_publisher.publish(manip_message)
            self.gripper_publisher.publish(gripper_message)
            return

        # elif self.manip_steering_mode == self.ManipSteeringMode.ANOTHER_MODE:
        #     pass

    def change_mode(self, buttons_values: dict):
        if self.steered_object == self.SteeredObject.ROVER:
            if buttons_values['A_button']:
                self.stop_object()
                self.rover_steering_mode = self.RoverSteeringMode.NORMAL
                return True
            if buttons_values['B_button']:
                self.stop_object()
                self.rover_steering_mode = self.RoverSteeringMode.TANK
                return True
            if buttons_values['X_button']:
                self.stop_object()
                self.rover_steering_mode = self.RoverSteeringMode.GAMER
                return True
        if self.steered_object == self.SteeredObject.MANIP:
            return True
            # if buttons_values['A_button']:
            #     self.stop_object()
            #     self.manip_steering_mode = self.ManipSteeringMode.ANOTHER_MODE
            #     return True
            # if buttons_values['X_button']:
            #     self.stop_object()
            #     self.manip_steering_mode = self.ManipSteeringMode.INVERSE_KINEMATICS
            #     return True
        return False

    def stop_object(self):
        if self.steered_object == self.SteeredObject.ROVER:
            rover_message = Twist()
            self.rover_publisher.publish(rover_message)
        elif self.steered_object == self.SteeredObject.MANIP:
            manip_message = Twist()
            gripper_message = Float64()
            self.manip_publisher.publish(manip_message)
            self.gripper_publisher.publish(gripper_message)

    class SteeredObject(Enum):
        ROVER = 0
        MANIP = 1

    class RoverSteeringMode(Enum):
        NORMAL = 0
        TANK = 1
        GAMER = 2

    class ManipSteeringMode(Enum):
        INVERSE_KINEMATICS = 0
        # ANOTHER_MODE = 1

    # This class simulates dynamic system used for softening joystick's input signal
    class InertiaModel():

        def __init__(self, i1: float, i2: float):
            self.I1 = i1
            self.I2 = i2
            self.prev_y = 0
            self.prev_dy = 0

        def step(self, u: float):
            calc_dy = (u - self.prev_y) * self.I1
            ddy = calc_dy - self.prev_dy
            if (calc_dy > 0 and ddy > 0) or (calc_dy <= 0 and ddy <= 0):
                dy = calc_dy * self.I2 + self.prev_dy * (1 - self.I2)
            else:
                dy = calc_dy
            self.prev_dy = dy
            self.prev_y = self.prev_y + dy
            return self.prev_y


def nonlin_scale(value: float, params: dict) -> float:
    return value * params['scale_coefficient'] * abs(value)**(params['shape_coefficient'] - 1.0)


if __name__ == '__main__':
    try:
        JoystickController().run()
    except rospy.ROSInterruptException:
        pass
