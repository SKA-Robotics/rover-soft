#!/usr/bin/python3
from enum import Enum
from math import tan
from abc import abstractmethod, ABC
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# used topics names
ROVER_TOPIC_NAME = "/cmd_vel"
MANIP_TOPIC_NAME = "/cmd_manip"
GRIPPER_TOPIC_NAME = "/cmd_grip"


class JoystickController:

    def __init__(self) -> None:
        ID = rospy.get_param("~ID", "0")
        NAME = "joystick_steering_node_" + ID
        rospy.init_node(NAME)
        JOY_TOPIC_NAME = rospy.get_param("~joy_message_topic_name", "joy")

        # get axes and buttons configuration from the file
        JOYSTICK_TYPE = rospy.get_param("~joystick_type", "XBOX_XS")
        JOYSTICK_DATA = rospy.get_param(f"~{JOYSTICK_TYPE}", None)
        MODES_DATA = rospy.get_param("~steering_modes", None)
        self.AXES_ID: dict = JOYSTICK_DATA['axes']
        self.BUTTONS_ID: dict = JOYSTICK_DATA['buttons']

        # set rover steering and normal modes at begining
        self.rover = self.RoverSender(MODES_DATA['rover'])
        self.manip = self.ManipSender(MODES_DATA['manip'])
        self.active_object = self.rover

        rospy.Subscriber(JOY_TOPIC_NAME, Joy, self._joy_listener_callback)

    def __del__(self) -> None:
        # stopping rover or manipulator before killing the node
        self.active_object.stop_object()

    def run(self) -> None:
        rospy.spin()

    def _joy_listener_callback(self, data: Joy) -> None:
        # convert data to much easier use
        VALUES = dict((name, data.buttons[id])
                      for name, id in self.BUTTONS_ID.items())
        VALUES.update(dict((name, data.axes[id])
                           for name, id in self.AXES_ID.items()))

        # change currently steered object and stop the previous one
        if VALUES['start_button'] and not VALUES['back_button']:
            self.active_object.stop_object()
            self.active_object = self.rover
            return
        if VALUES['back_button'] and not VALUES['start_button']:
            self.active_object.stop_object()
            self.active_object = self.manip
            return

        # change steering mode and stop object to avoid unexpected behaviour
        if self.active_object.change_mode(VALUES):
            return

        # all values are scaled in range (-1,1) corresponding to a percentage of maximal speed in both sides
        self.active_object.steer_object(VALUES)

    class SteeringMessageSender(ABC):

        @abstractmethod
        def steer_object(self, values: dict) -> None:
            pass

        @abstractmethod
        def change_mode(self, buttons_values: dict) -> None:
            pass

        @abstractmethod
        def stop_object(self) -> None:
            pass

    class RoverSender(SteeringMessageSender):

        class RoverSteeringMode(Enum):
            NORMAL = 0
            TANK = 1
            GAMER = 2

        def __init__(self, modes_data: dict) -> None:
            self.MODES_DATA = modes_data
            self.steering_mode = self.RoverSteeringMode.NORMAL
            self.rover_publisher = rospy.Publisher(
                ROVER_TOPIC_NAME, Twist, queue_size=10)

            # used in child mode
            self.CHILD_MODE = rospy.get_param("~child_mode", False)
            if self.CHILD_MODE:
                i1 = min(1.0, max(rospy.get_param(
                    "~child_mode_inertia_i1", 0.4444), 0.0))
                i2 = min(1.0, max(rospy.get_param(
                    "~child_mode_inertia_i2", 0.0066), 0.0))
                self.lin_inertia = self.InertiaModel(i1, i2)
                self.ang_inertia = self.InertiaModel(i1, i2)
                rospy.loginfo("Child mode enabled")

        def steer_object(self, values: dict) -> None:
            rover_message = Twist()
            PARAMS = self.MODES_DATA[self.steering_mode.name.lower()]
            lin_cmd = 0
            ang_cmd = 0

            if self.steering_mode == self.RoverSteeringMode.NORMAL:
                lin_cmd = nonlin_scale(
                    values['right_stick_vertical'], PARAMS['linear'])
                ang_cmd = nonlin_scale(
                    values['right_stick_horizontal'], PARAMS['angular'])

            elif self.steering_mode == self.RoverSteeringMode.TANK:
                lin_cmd = nonlin_scale(
                    (values['left_stick_vertical'] + values['right_stick_vertical']) / 2, PARAMS)
                ang_cmd = nonlin_scale(
                    (values['right_stick_vertical'] - values['left_stick_vertical']) / 2, PARAMS)

            elif self.steering_mode == self.RoverSteeringMode.GAMER:
                lin_cmd = nonlin_scale(
                    (values['right_trigger'] - values['left_trigger']) / 2, PARAMS['linear'])
                turning_angle = nonlin_scale(
                    values['right_stick_horizontal'], PARAMS['angular'])
                ang_cmd = lin_cmd * tan(turning_angle)

            if (self.CHILD_MODE):
                rover_message.linear.x = self.lin_inertia.step(lin_cmd)
                rover_message.angular.z = self.ang_inertia.step(ang_cmd)
            else:
                rover_message.linear.x = lin_cmd
                rover_message.angular.z = ang_cmd

            self.rover_publisher.publish(rover_message)

        def change_mode(self, buttons_values: dict) -> None:
            if buttons_values['A_button']:
                self.stop_object()
                self.steering_mode = self.RoverSteeringMode.NORMAL
                return True
            if buttons_values['B_button']:
                self.stop_object()
                self.steering_mode = self.RoverSteeringMode.TANK
                return True
            if buttons_values['X_button']:
                self.stop_object()
                self.steering_mode = self.RoverSteeringMode.GAMER
                return True
            return False

        def stop_object(self) -> None:
            rover_message = Twist()
            self.rover_publisher.publish(rover_message)

        # This class simulates dynamic system used for softening joystick's input signal
        class InertiaModel:

            def __init__(self, i1: float, i2: float) -> None:
                self.I1 = i1
                self.I2 = i2
                self.prev_y = 0
                self.prev_dy = 0

            def step(self, u: float) -> None:
                calc_dy = (u - self.prev_y) * self.I1
                ddy = calc_dy - self.prev_dy
                if (calc_dy > 0 and ddy > 0) or (calc_dy <= 0 and ddy <= 0):
                    dy = calc_dy * self.I2 + self.prev_dy * (1 - self.I2)
                else:
                    dy = calc_dy
                self.prev_dy = dy
                self.prev_y = self.prev_y + dy
                return self.prev_y

    class ManipSender(SteeringMessageSender):

        class ManipSteeringMode(Enum):
            INVERSE_KINEMATICS = 0
            # ANOTHER_MODE = 1

        def __init__(self, modes_data: dict) -> None:
            self.MODES_DATA = modes_data
            self.steering_mode = self.ManipSteeringMode.INVERSE_KINEMATICS
            self.manip_publisher = rospy.Publisher(
                MANIP_TOPIC_NAME, Twist, queue_size=10)
            self.gripper_publisher = rospy.Publisher(
                GRIPPER_TOPIC_NAME, Float64, queue_size=10)

        def steer_object(self, values: dict) -> None:
            PARAMS = self.MODES_DATA[self.steering_mode.name.lower()]

            if self.steering_mode == self.ManipSteeringMode.INVERSE_KINEMATICS:
                manip_message = Twist()
                manip_message.linear.x = nonlin_scale(
                    values['right_stick_vertical'], PARAMS['x'])
                manip_message.linear.y = nonlin_scale(
                    values['right_stick_horizontal'], PARAMS['y'])
                manip_message.linear.z = nonlin_scale(
                    values['left_stick_vertical'], PARAMS['z'])
                manip_message.angular.x = nonlin_scale(
                    values['cross_horizontal'], PARAMS['roll'])
                manip_message.angular.y = nonlin_scale(
                    values['left_stick_horizontal'], PARAMS['pitch'])
                clamp = nonlin_scale(
                    values['left_trigger'] - values['right_trigger'], PARAMS['gripper'])
                gripper_message = Float64(clamp)

                self.manip_publisher.publish(manip_message)
                self.gripper_publisher.publish(gripper_message)

            # elif self.manip_steering_mode == self.ManipSteeringMode.ANOTHER_MODE:
            #     pass

        def change_mode(self, buttons_values: dict) -> None:
            # if buttons_values['A_button']:
            #     self.stop_object()
            #     self.manip_steering_mode = self.ManipSteeringMode.ANOTHER_MODE
            #     return True
            # if buttons_values['X_button']:
            #     self.stop_object()
            #     self.manip_steering_mode = self.ManipSteeringMode.INVERSE_KINEMATICS
            #     return True
            return False

        def stop_object(self) -> None:
            manip_message = Twist()
            gripper_message = Float64()
            self.manip_publisher.publish(manip_message)
            self.gripper_publisher.publish(gripper_message)


def nonlin_scale(value: float, params: dict) -> float:
    return value * params['scale_coefficient'] * abs(value)**(params['shape_coefficient'] - 1.0)


if __name__ == '__main__':
    try:
        JoystickController().run()
    except rospy.ROSInterruptException:
        pass
