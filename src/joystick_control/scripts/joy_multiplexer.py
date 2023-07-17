#!/usr/bin/python3
import roslib

roslib.load_manifest("joystick_control")

import rospy
from threading import Lock
from functools import partial

from std_msgs.msg import String
from sensor_msgs.msg import Joy
from joystick_control.srv import SelectJoy, AddJoy, RemoveJoy, ListJoy

from utils.translate_joystick import JoystickTranslator


class JoystickMultiplexer:
    def __init__(self) -> None:
        rospy.init_node("joy_multiplexer")

        self.STEERING_MODES = rospy.get_param("~steering_modes", None)

        self.active_joystick = ""
        self.joystick_subscribers = {}
        self.active_topic = ""
        self.output_publishers = {
            name: rospy.Publisher(name, Joy, queue_size=10)
            for name in self.STEERING_MODES.keys()
            if name != "emergency"
        }

        self.selected_joy_topic = rospy.Publisher("selected_joy", String, queue_size=10)
        rospy.Service("~select_joy", SelectJoy, self._select_joy)
        rospy.Service("~list_joy", ListJoy, self._list_joy)  # TODO: correct type
        rospy.Service("~add_joy", AddJoy, self._add_joy)
        rospy.Service("~remove_joy", RemoveJoy, self._remove_joy)

        # required, because services run on different threads and have to be thread-safe
        self.service_lock = Lock()

        self.translator = JoystickTranslator()

    def run(self) -> None:
        rospy.spin()

    def _joy_subscriber_callback(self, data: Joy, topic_name=""):
        inputs = self.translator.translate(data)

        with self.service_lock:
            # emergency stop
            if inputs[self.STEERING_MODES["emergency"]["button"]] == 1:
                rospy.logwarn("emergency stop")
                self.active_joystick = "__none"
                self.active_topic = "__none"
                return

            # controller mode selection
            for config in self.STEERING_MODES.values():
                if inputs[config["button"]] == 1:
                    rospy.logwarn(
                        f"enabling {config['topic']} with joystick {topic_name}"
                    )
                    self.active_joystick = topic_name
                    self.active_topic = config["topic"]

            # input handling
            if topic_name == self.active_joystick:
                self.output_publishers[self.active_topic].publish(data)

    def _select_joy(self, req):
        topic_name = req.topic_name
        rospy.loginfo(f"Selecting {topic_name}")
        with self.service_lock:
            # allow empty name to disable control
            if topic_name == "__none":
                self.active_joystick = "__none"
                self.active_topic = "__none"
                return True

            # only allow topics that have been added already
            if topic_name not in self.joystick_subscribers:
                return False

            self.active_joystick = topic_name
            return True

    def _add_joy(self, req):
        topic_name = req.topic_name
        rospy.loginfo(f"Adding {topic_name}")
        with self.service_lock:
            # prevent topic collisions and __none, which is reserved
            if topic_name in self.joystick_subscribers or topic_name == "__none":
                return False

            self.joystick_subscribers[topic_name] = rospy.Subscriber(
                topic_name,
                Joy,
                partial(self._joy_subscriber_callback, topic_name=topic_name),
            )
            return True

    def _remove_joy(self, req):
        topic_name = req.topic_name
        rospy.loginfo(f"Removing {topic_name}")
        with self.service_lock:
            # don't remove topics that don't exist
            if topic_name not in self.joystick_subscribers:
                return False

            self.joystick_subscribers[topic_name].unregister()
            del self.joystick_subscribers[topic_name]

            return True

    def _list_joy(self, req):
        with self.service_lock:
            # create a copy to avoid race conditions
            return self.joystick_subscribers.copy()


if __name__ == "__main__":
    try:
        JoystickMultiplexer().run()
    except rospy.ROSInterruptException:
        pass
