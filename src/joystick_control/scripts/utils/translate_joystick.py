import rospy

from sensor_msgs.msg import Joy


class JoystickTranslator:
    def __init__(self):
        JOYSTICK_TYPE = rospy.get_param("~joystick_type", "XBOX_XS")
        JOYSTICK_DATA = rospy.get_param(f"~{JOYSTICK_TYPE}", None)
        self.AXES_ID: dict = JOYSTICK_DATA["axes"]
        self.BUTTONS_ID: dict = JOYSTICK_DATA["buttons"]

    def translate(self, data: Joy):
        inputs = dict((name, data.buttons[id]) for name, id in self.BUTTONS_ID.items())
        inputs.update(dict((name, data.axes[id]) for name, id in self.AXES_ID.items()))

        return inputs
