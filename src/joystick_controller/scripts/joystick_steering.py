#!/usr/bin/python3
from enum import Enum
from math import tan
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist

# used topics names
ROVER_TOPIC_NAME = "/cmd_vel"
MANIP_TOPIC_NAME = "/cmd_manip"
GRIPPER_TOPIC_NAME = "/cmd_grip"

# names to use in messages
LIMBS_NAMES = [
    'arm_rotate',
    'arm_lift',
    'claw_rotate',
    'claw_lift',
    'arm_tilt',
    'claw_clamp',
]


class JoystickController:

    def __init__(self):
        ID = rospy.get_param("~ID", "0")
        NAME = "joystick_steering_node_" + ID
        rospy.init_node(NAME)
        JOY_TOPIC_NAME = rospy.get_param("~joy_message_topic_name", "joy")

        # set rover steering and normal modes at begining
        self.steered_object = self.SteeredObject.ROVER
        self.rover_steering_mode = self.RoverSteeringMode.NORMAL
        self.manip_steering_mode = self.ManipSteeringMode.NORMAL

        # get axes and buttons configuration from the file
        JOYSTICK_TYPE = rospy.get_param("~joystick_type", "XBOX_XS")
        JOYSTICK_DATA = rospy.get_param(f"~{JOYSTICK_TYPE}", None)
        self.MODES_DATA = rospy.get_param("~steering_modes", None)
        self.AXES_ID = JOYSTICK_DATA['axes']
        self.BUTTONS_ID = JOYSTICK_DATA['buttons']

        self.rover_publisher = rospy.Publisher(ROVER_TOPIC_NAME, Twist, queue_size=10)
        # self.manip_publisher = rospy.Publisher(MANIP_TOPIC_NAME, JointState, queue_size=10)
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
            lin_cmd = values['right_stick_vertical']
            ang_cmd = values['right_stick_horizontal']
            lin_cmd *= PARAMS['linear']['scale_coefficient'] * abs(lin_cmd)**(PARAMS['linear']['shape_coefficient'] -
                                                                              1.0)
            ang_cmd *= PARAMS['angular']['scale_coefficient'] * abs(ang_cmd)**(PARAMS['angular']['shape_coefficient'] -
                                                                               1.0)

        elif self.rover_steering_mode == self.RoverSteeringMode.TANK:
            lin_cmd = (values['left_stick_vertical'] + values['right_stick_vertical']) / 2
            ang_cmd = (values['right_stick_vertical'] - values['left_stick_vertical']) / 2
            lin_cmd *= PARAMS['scale_coefficient'] * abs(lin_cmd)**(PARAMS['shape_coefficient'] - 1.0)
            ang_cmd *= PARAMS['scale_coefficient'] * abs(ang_cmd)**(PARAMS['shape_coefficient'] - 1.0)

        elif self.rover_steering_mode == self.RoverSteeringMode.GAMER:
            lin_cmd = (values['right_trigger'] - values['left_trigger']) / 2
            turning_angle = values['right_stick_horizontal']  # value of angle is relative (0,1), NOT in rad
            lin_cmd *= PARAMS['linear']['scale_coefficient'] * abs(lin_cmd)**(PARAMS['linear']['shape_coefficient'] -
                                                                              1.0)
            turning_angle *= PARAMS['angular']['scale_coefficient'] * abs(turning_angle)**(
                PARAMS['angular']['shape_coefficient'] - 1.0)
            ang_cmd = lin_cmd * tan(turning_angle)

        if (self.child_mode):
            rover_message.linear.x = self.lin_inertia.step(lin_cmd)
            rover_message.angular.z = self.ang_inertia.step(ang_cmd)
        else:
            rover_message.linear.x = lin_cmd
            rover_message.angular.z = ang_cmd

        self.rover_publisher.publish(rover_message)

    def steer_manip(self, values: dict):
        # effort = dict()
        # if self.manip_steering_mode == self.ManipSteeringMode.NORMAL:
        #     effort['arm_rotate'] = -values['left_stick_horizontal']
        #     effort['arm_lift'] = -values['left_stick_vertical']
        #     effort['claw_rotate'] = values['right_stick_horizontal']
        #     effort['claw_lift'] = -values['right_stick_vertical']
        #     effort['arm_tilt'] = -(values['left_trigger'] - values['right_trigger']) / 2
        #     # each bumper has range (0,1)
        #     effort['claw_clamp'] = (values['left_bumper'] - values['right_bumper'])

        # elif self.manip_steering_mode == self.ManipSteeringMode.GAMER:
        #     effort['arm_rotate'] = values['right_stick_horizontal']
        #     effort['arm_lift'] = -values['right_stick_vertical']
        #     effort['claw_rotate'] = -values['left_stick_horizontal']
        #     effort['claw_lift'] = -values['left_stick_vertical']
        #     effort['arm_tilt'] = -values['cross_vertical']
        #     # each bumper has range (0,1)
        #     effort['claw_clamp'] = (values['left_bumper'] - values['right_bumper'])

        if self.manip_steering_mode == self.ManipSteeringMode.INVERSE_KINEMATICS:
            manip_message = Twist()
            PARAMS = self.MODES_DATA['manip'][self.manip_steering_mode.name.lower()]
            manip_message.linear.x = values['cross_horizontal']
            manip_message.linear.y = values['cross_vertical']
            manip_message.linear.z = values['right_stick_vertical']
            manip_message.angular.y = values['left_stick_vertical']  # pitch
            manip_message.angular.x = values['left_stick_horizontal']  # roll
            gripper_message = Float64(values['left_trigger'] - values['right_trigger'])

            manip_message.linear.x *= PARAMS['x']['scale_coefficient'] * abs(
                manip_message.linear.x)**(PARAMS['x']['shape_coefficient'] - 1.0)
            manip_message.linear.y *= PARAMS['y']['scale_coefficient'] * abs(
                manip_message.linear.y)**(PARAMS['y']['shape_coefficient'] - 1.0)
            manip_message.linear.z *= PARAMS['z']['scale_coefficient'] * abs(
                manip_message.linear.z)**(PARAMS['z']['shape_coefficient'] - 1.0)
            manip_message.angular.x *= PARAMS['roll']['scale_coefficient'] * abs(
                manip_message.angular.x)**(PARAMS['roll']['shape_coefficient'] - 1.0)
            manip_message.angular.y *= PARAMS['pitch']['scale_coefficient'] * abs(
                manip_message.angular.y)**(PARAMS['pitch']['shape_coefficient'] - 1.0)
            gripper_message.data *= PARAMS['gripper']['scale_coefficient'] * abs(
                gripper_message.data)**(PARAMS['gripper']['shape_coefficient'] - 1.0)

            self.manip_publisher.publish(manip_message)
            self.gripper_publisher.publish(gripper_message)
            return

        # manip_message = JointState()
        # manip_message.header.stamp = rospy.get_rostime()
        # manip_message.name = list()
        # manip_message.effort = list()
        # for name, params in self.MODES_DATA['manip'][self.manip_steering_mode.name.lower()].items():
        #     manip_message.name.append(name)
        #     manip_message.effort.append(params['scale_coefficient'] * effort[name] *
        #                                 abs(effort[name])**(params['shape_coefficient'] - 1.0))
        # self.manip_publisher.publish(manip_message)

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
            if buttons_values['A_button']:
                self.stop_object()
                self.manip_steering_mode = self.ManipSteeringMode.NORMAL
                return True
            if buttons_values['B_button']:
                self.stop_object()
                self.manip_steering_mode = self.ManipSteeringMode.GAMER
                return True
            if buttons_values['X_button']:
                self.stop_object()
                self.manip_steering_mode = self.ManipSteeringMode.INVERSE_KINEMATICS
                return True
        return False

    def stop_object(self):
        if self.steered_object == self.SteeredObject.ROVER:
            rover_message = Twist()
            self.rover_publisher.publish(rover_message)
        elif self.steered_object == self.SteeredObject.MANIP:
            # manip_message = JointState()
            # manip_message.header.stamp = rospy.get_rostime()
            # manip_message.name = LIMBS_NAMES
            # manip_message.effort = [0] * 6
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
        NORMAL = 0
        GAMER = 1
        INVERSE_KINEMATICS = 2  # comming soon...

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


if __name__ == '__main__':
    try:
        JoystickController().run()
    except rospy.ROSInterruptException:
        pass
