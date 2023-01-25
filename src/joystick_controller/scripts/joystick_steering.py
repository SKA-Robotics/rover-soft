#!/usr/bin/python3
from enum import Enum
from math import tan
import rospy
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist

# used topics names
ROVER_TOPIC_NAME = "/cmd_vel"
MANIP_TOPIC_NAME = "/cmd_manip"

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
        self.steered_object = self.SteeredObject.rover
        self.rover_steering_mode = self.RoverSteeringMode.normal
        self.manip_steering_mode = self.ManipSteeringMode.normal

        # get axes and buttons configuration from the file
        JOYSTICK_TYPE = rospy.get_param("~joystick_type", "XBOX_XS")
        JOYSTICK_DATA = rospy.get_param(f"~{JOYSTICK_TYPE}", None)
        self.MODES_DATA = rospy.get_param("~steering_modes", None)
        self.AXES_ID = JOYSTICK_DATA['axes']
        self.BUTTONS_ID = JOYSTICK_DATA['buttons']

        self.rover_publisher = rospy.Publisher(ROVER_TOPIC_NAME, Twist, queue_size=10)
        self.manip_publisher = rospy.Publisher(MANIP_TOPIC_NAME, JointState, queue_size=10)
        rospy.Subscriber(JOY_TOPIC_NAME, Joy, self.joy_listener_callback)

        # used in child mode
        self.child_mode = rospy.get_param("~child_mode", False)
        if self.child_mode:
            i1 = min(1.0, max(rospy.get_param("~child_mode_inertia_i1", 0.4444), 0.0))
            i2 = min(1.0, max(rospy.get_param("~child_mode_inertia_i2", 0.0066), 0.0))
            self.linx_inertia = self.InertiaModel(i1, i2)
            self.liny_inertia = self.InertiaModel(i1, i2)
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
            self.steered_object = self.SteeredObject.rover
            return
        if VALUES['back_button'] and not VALUES['start_button']:
            self.stop_object()
            self.steered_object = self.SteeredObject.manip
            return

        # change steering mode and stop object to avoid unexpected behaviour
        if self.change_mode(VALUES):
            return

        # all velocities are scaled in range (-1,1) corresponding to a percentage of maximal speed in both sides
        if self.steered_object == self.SteeredObject.rover:
            self.steer_rover(VALUES)
        elif self.steered_object == self.SteeredObject.manip:
            self.steer_manip(VALUES)

    def steer_rover(self, values):
        rover_message = Twist()
        PARAMS = self.MODES_DATA['rover'][self.rover_steering_mode.name]
        linx_cmd = 0
        liny_cmd = 0
        ang_cmd = 0

        if self.rover_steering_mode == self.RoverSteeringMode.normal:
            linx_cmd = values['right_stick_vertical']
            ang_cmd = values['right_stick_horizontal']
            linx_cmd *= PARAMS['linear']['scale_coefficient'] * abs(linx_cmd)**(PARAMS['linear']['shape_coefficient'] -
                                                                              1.0)
            ang_cmd *= PARAMS['angular']['scale_coefficient'] * abs(ang_cmd)**(PARAMS['angular']['shape_coefficient'] -
                                                                               1.0)

        elif self.rover_steering_mode == self.RoverSteeringMode.tank:
            linx_cmd = (values['left_stick_vertical'] + values['right_stick_vertical']) / 2
            ang_cmd = (values['right_stick_vertical'] - values['left_stick_vertical']) / 2
            linx_cmd *= PARAMS['scale_coefficient'] * abs(linx_cmd)**(PARAMS['shape_coefficient'] - 1.0)
            ang_cmd *= PARAMS['scale_coefficient'] * abs(ang_cmd)**(PARAMS['shape_coefficient'] - 1.0)

        elif self.rover_steering_mode == self.RoverSteeringMode.gamer:
            linx_cmd = (values['right_trigger'] - values['left_trigger']) / 2
            turning_angle = values['right_stick_horizontal']  # value of angle is relative (0,1), NOT in rad
            linx_cmd *= PARAMS['linear']['scale_coefficient'] * abs(linx_cmd)**(PARAMS['linear']['shape_coefficient'] -
                                                                              1.0)
            turning_angle *= PARAMS['angular']['scale_coefficient'] * abs(turning_angle)**(
                PARAMS['angular']['shape_coefficient'] - 1.0)
            ang_cmd = linx_cmd * tan(turning_angle)
        
        elif self.rover_steering_mode == self.RoverSteeringMode.holonomic:
            linx_cmd = values['left_stick_vertical']
            liny_cmd = values['left_stick_horizontal']
            ang_cmd = values['right_stick_horizontal']
            linx_cmd *= PARAMS['linear']['scale_coefficient'] * abs(linx_cmd)**(PARAMS['linear']['shape_coefficient'] -
                                                                              1.0)
            liny_cmd *= PARAMS['linear']['scale_coefficient'] * abs(liny_cmd)**(PARAMS['linear']['shape_coefficient'] -
                                                                              1.0)
            ang_cmd *= PARAMS['angular']['scale_coefficient'] * abs(ang_cmd)**(PARAMS['angular']['shape_coefficient'] -
                                                                               1.0)

        if (self.child_mode):
            rover_message.linear.x = self.linx_inertia.step(linx_cmd)
            rover_message.linear.y = self.liny_inertia.step(liny_cmd)
            rover_message.angular.z = self.ang_inertia.step(ang_cmd)
        else:
            rover_message.linear.x = linx_cmd
            rover_message.linear.y = liny_cmd
            rover_message.angular.z = ang_cmd
        self.rover_publisher.publish(rover_message)

    def steer_manip(self, values):
        effort = dict()
        if self.manip_steering_mode == self.ManipSteeringMode.normal:
            effort['arm_rotate'] = -values['left_stick_horizontal']
            effort['arm_lift'] = -values['left_stick_vertical']
            effort['claw_rotate'] = values['right_stick_horizontal']
            effort['claw_lift'] = -values['right_stick_vertical']
            effort['arm_tilt'] = -(values['left_trigger'] - values['right_trigger']) / 2
            # each bumper has range (0,1)
            effort['claw_clamp'] = (values['left_bumper'] - values['right_bumper'])

        elif self.manip_steering_mode == self.ManipSteeringMode.gamer:
            effort['arm_rotate'] = values['right_stick_horizontal']
            effort['arm_lift'] = -values['right_stick_vertical']
            effort['claw_rotate'] = -values['left_stick_horizontal']
            effort['claw_lift'] = -values['left_stick_vertical']
            effort['arm_tilt'] = -values['cross_vertical']
            # each bumper has range (0,1)
            effort['claw_clamp'] = (values['left_bumper'] - values['right_bumper'])

        elif self.manip_steering_mode == self.ManipSteeringMode.inverse_kinematics:
            # velocities = dict()
            # velocities['x'] = VALUES['cross_horizontal']
            # velocities['y'] = VALUES['cross_vertical']
            # velocities['z'] = VALUES['right_stick_vertical']
            # velocities['theta'] = VALUES['left_stick_vertical']
            # velocities['fi'] = VALUES['left_stick_horizontal']
            # velocities['clamp'] = (VALUES['left_trigger'] - VALUES['right_trigger'])
            # manip_message = JointState()
            # manip_message.header.stamp = rospy.get_rostime()
            # manip_message.name = list()
            # manip_message.velocity = list()
            # for name, params in self.MODES_DATA['manip'][self.manip_steering_mode.name].items():
            #     manip_message.name.append(name)
            #     manip_message.velocity.append(params['scale_coefficient'] * velocities(name) *
            #                                   abs(velocities(name))**(params['shape_coefficient'] - 1.0))
            # # publish message
            return

        manip_message = JointState()
        manip_message.header.stamp = rospy.get_rostime()
        manip_message.name = list()
        manip_message.effort = list()
        for name, params in self.MODES_DATA['manip'][self.manip_steering_mode.name].items():
            manip_message.name.append(name)
            manip_message.effort.append(params['scale_coefficient'] * effort(name) *
                                        abs(effort(name))**(params['shape_coefficient'] - 1.0))
        self.manip_publisher.publish(manip_message)

    def change_mode(self, buttons_values):
        if self.steered_object == self.SteeredObject.rover:
            if buttons_values['A_button']:
                self.stop_object()
                self.rover_steering_mode = self.RoverSteeringMode.normal
                return True
            if buttons_values['B_button']:
                self.stop_object()
                self.rover_steering_mode = self.RoverSteeringMode.tank
                return True
            if buttons_values['X_button']:
                self.stop_object()
                self.rover_steering_mode = self.RoverSteeringMode.gamer
                return True
            if buttons_values['Y_button']:
                self.stop_object()
                self.rover_steering_mode = self.RoverSteeringMode.holonomic
                return True
        if self.steered_object == self.SteeredObject.manip:
            if buttons_values['A_button']:
                self.stop_object()
                self.manip_steering_mode = self.ManipSteeringMode.normal
                return True
            if buttons_values['B_button']:
                self.stop_object()
                self.manip_steering_mode = self.ManipSteeringMode.gamer
                return True
            if buttons_values['X_button']:
                self.stop_object()
                self.manip_steering_mode = self.ManipSteeringMode.inverse_kinematics
                return True
        return False

    def stop_object(self):
        if self.steered_object == self.SteeredObject.rover:
            rover_message = Twist()
            self.rover_publisher.publish(rover_message)
        elif self.steered_object == self.SteeredObject.manip:
            manip_message = JointState()
            manip_message.header.stamp = rospy.get_rostime()
            manip_message.name = LIMBS_NAMES
            manip_message.effort = [0] * 6
            self.manip_publisher.publish(manip_message)

    class SteeredObject(Enum):
        rover = 0
        manip = 1

    class RoverSteeringMode(Enum):
        normal = 0
        tank = 1
        gamer = 2
        holonomic = 3

    class ManipSteeringMode(Enum):
        normal = 0
        gamer = 1
        inverse_kinematics = 2  # comming soon...

    # This class simulates dynamic system used for softening joystick's input signal
    class InertiaModel():

        def __init__(self, i1, i2):
            self.i1 = i1
            self.i2 = i2
            self.prev_y = 0
            self.prev_dy = 0

        def step(self, u):
            calc_dy = (u - self.prev_y) * self.i1
            ddy = calc_dy - self.prev_dy
            if (calc_dy > 0 and ddy > 0) or (calc_dy <= 0 and ddy <= 0):
                dy = calc_dy * self.i2 + self.prev_dy * (1 - self.i2)
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