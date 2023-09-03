#!/usr/bin/python3
import roslib

roslib.load_manifest("joystick_control")

import rospy
from threading import Lock
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from joystick_control.msg import Gamepad
from joystick_control.cfg import Joy5dofManipulatorConfig
from topic_tools.srv import MuxSelect

from utils.translate_joystick import JoystickTranslator
from utils.axis_transformations import deadzone


class Joystick5dofManipulator:
    def __init__(self) -> None:
        rospy.init_node("joy_5dof_manipulator")
        self.CHANGE_MODE_BUTTON = rospy.get_param("~change_mode_button", None)
        self.MAX_JOINT_EFFORT = rospy.get_param("~max_joint_effort", None)
        self.MAX_LINEAR_RATE = rospy.get_param("~max_linear_rate", None)
        self.MAX_ANGULAR_RATE = rospy.get_param("~max_angular_rate", None)

        # topics for main processing
        self.subscriber = rospy.Subscriber(
            "joy_5dof_manipulator",
            Gamepad,
            self._joy_subscriber_callback,
        )
        self.ik_publisher = rospy.Publisher("/cmd_manip", Twist, queue_size=10)
        self.fk_publisher = rospy.Publisher(
            "/joy_5dof_manipulator/manip_command", JointState, queue_size=10
        )

        self.multiplexer_select_service = rospy.ServiceProxy(
            "/manip_command_mux/select", MuxSelect
        )

        self.translator = JoystickTranslator()

        # key debouncing
        self.prev_inputs = {key: 0 for key in self.translator.BUTTONS_ID.keys()}

        # dynamic parameters
        self.mode = 0
        self.gear = 1
        self.gear_max_speeds = {
            1: 10,
            2: 50,
            3: 100,
        }

        # dynamic_reconfigure server
        self.config_lock = Lock()
        self.config_server = Server(Joy5dofManipulatorConfig, self._dynamic_reconfigure)
        self.config_client = Client("joy_5dof_manipulator", timeout=30)

        self.MODES = next(
            (
                {
                    entry["name"]: entry["value"]
                    for entry in eval(param.edit_method)["enum"]
                }
                for param in self.config_server.description.groups[0].parameters
                if param.name == "mode"
            ),
            {},
        )

    def run(self) -> None:
        rospy.spin()

    def _joy_subscriber_callback(self, data: Gamepad):
        inputs = self.translator.translate(data)

        with self.config_lock:
            # gear up button
            if inputs["right_bumper"] == 1 and self.prev_inputs["right_bumper"] == 0:
                self.config_lock.release()
                self.config_client.update_configuration({"gear": self.gear + 1})
                self.config_lock.acquire()

            # gear down button
            if inputs["left_bumper"] == 1 and self.prev_inputs["left_bumper"] == 0:
                self.config_lock.release()
                self.config_client.update_configuration({"gear": self.gear - 1})
                self.config_lock.acquire()

            # mode change
            if (
                inputs[self.CHANGE_MODE_BUTTON] == 1
                and self.prev_inputs[self.CHANGE_MODE_BUTTON] == 0
            ):
                next_mode = self.mode + 1
                if next_mode > self.config_server.type.max["mode"]:
                    next_mode = self.config_server.type.min["mode"]

                self.config_lock.release()
                self.config_client.update_configuration({"mode": next_mode})
                self.config_lock.acquire()

            multiplier = self.gear_max_speeds[self.gear] / 100
            effort_multiplier = self.MAX_JOINT_EFFORT * multiplier
            linear_multiplier = self.MAX_LINEAR_RATE * multiplier
            angular_multiplier = self.MAX_ANGULAR_RATE * multiplier

            # calculate movement based on the current mode
            if self.mode == self.MODES["forward"]:
                message = JointState()

                message.header.stamp = rospy.Time.now()
                message.name = [
                    "base_cyl",
                    "cyl_arm1",
                    "arm1_arm2",
                    "arm2_arm3",
                    "arm3_tool",
                ]
                message.effort = [
                    -1 * deadzone(inputs["left_stick_horizontal"], 0.15),
                    (inputs["left_trigger"] - inputs["right_trigger"]),
                    -1 * deadzone(inputs["left_stick_vertical"], 0.15),
                    -1 * deadzone(inputs["right_stick_vertical"], 0.15),
                    -1 * deadzone(inputs["right_stick_horizontal"], 0.15),
                ]

                message.effort = [
                    effort * effort_multiplier for effort in message.effort
                ]

                self.fk_publisher.publish(message)

            if self.mode == self.MODES["inverse"]:
                message = Twist()

                message.linear.x = -1 * deadzone(inputs["left_stick_vertical"], 0.15)
                message.linear.y = -1 * deadzone(inputs["right_stick_horizontal"], 0.15)
                message.linear.z = -1 * deadzone(inputs["right_stick_vertical"], 0.15)

                message.angular.x = -1 * deadzone(inputs["left_stick_horizontal"], 0.15)
                message.angular.y = inputs["right_trigger"] - inputs["left_trigger"]

                message.linear.x *= linear_multiplier
                message.linear.y *= linear_multiplier
                message.linear.z *= linear_multiplier

                message.angular.x *= angular_multiplier
                message.angular.y *= angular_multiplier
                self.ik_publisher.publish(message)

        self.prev_inputs = inputs

    def _dynamic_reconfigure(self, config, level):
        if hasattr(self, "MODES"):
            if config["mode"] == self.MODES["forward"]:
                self.multiplexer_select_service("/joy_5dof_manipulator/manip_command")
            if config["mode"] == self.MODES["inverse"]:
                self.multiplexer_select_service("/manip_controller/manip_command")

        with self.config_lock:
            self.mode = config["mode"]
            self.gear = config["gear"]
            self.gear_max_speeds = {
                1: config["gear_1_max_speed"],
                2: config["gear_2_max_speed"],
                3: config["gear_3_max_speed"],
            }

        return config


if __name__ == "__main__":
    try:
        Joystick5dofManipulator().run()
    except rospy.ROSInterruptException:
        pass
