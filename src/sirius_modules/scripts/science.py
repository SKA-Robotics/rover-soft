#!/usr/bin/python3
import rospy
import math
import serial
from sirius_msgs.msg import JointState
from std_msgs.msg import Bool

FLASK_PER_SAMPLE = 4
SECTION_SIZE = math.pi / 2

MIXING_PASSES = 3
MIXING_OFFSET = math.pi / 4
LITMUS_OFFSET = math.pi / 12
INJECTOR_ROTATION = 10

SAMPLE = 0
INJECT = math.pi * 1/2
MIX = math.pi
LITMUS = math.pi * 3/2
INSPECT = math.pi

CAROUSEL_ROBOCLAW_TOPIC = "/roboclaw_driver/131/motor1/set_joint_state"
CAROUSEL_POSITION_TOPIC = "/roboclaw_driver/131/motor1/joint_state"
INJECTOR_ROBOCLAW_TOPIC = "/roboclaw_driver/131/motor2/set_joint_state"
INJECTOR_POSITION_TOPIC = "/roboclaw_driver/131/motor2/joint_state"
POSITION_EPS = 0.05

SERIAL_PORT_NAME = "/dev/ttyACM0"
BAUDRATE = 9600
CMD_SUCK = 105
CMD_OPEN = 0x6a
CMD_CLOSE = 0x6b
CMD_POSE = [0x6c, 0x6d, 0x6e, 0x6f]

class ScienceHardware:
    def __init__(self):
        self._serial = serial.Serial(SERIAL_PORT_NAME, BAUDRATE)
        self.carousel_position = 0
        self.injector_position = 0
        self.carousel_roboclaw = rospy.Publisher(CAROUSEL_ROBOCLAW_TOPIC, JointState, queue_size=10)
        rospy.Subscriber(CAROUSEL_POSITION_TOPIC, JointState, self.carousel_position_callback, queue_size=10)
        self.injector_roboclaw = rospy.Publisher(INJECTOR_ROBOCLAW_TOPIC, JointState, queue_size=10)
        rospy.Subscriber(INJECTOR_POSITION_TOPIC, JointState, self.injector_position_callback, queue_size=10)

    def get_carousel_position(self) -> float:
        return self.carousel_position

    def set_carousel_position(self, position: float):
        self.set_roboclaw_position(self.carousel_roboclaw, position)
        while abs(self.carousel_position - position) > POSITION_EPS:
            if rospy.is_shutdown():
                exit(1)

    def carousel_position_callback(self, msg):
        self.carousel_position = msg.position[0]

    def get_injector_position(self) -> float:
        return self.injector_position

    def set_injector_position(self, position: float):
        self.set_roboclaw_position(self.injector_roboclaw, position)
        while abs(self.injector_position - position) > POSITION_EPS and not rospy.is_shutdown():
            pass

    def injector_position_callback(self, msg):
        self.injector_position = msg.position[0]
    
    def set_roboclaw_position(self, publisher : rospy.Publisher, position: float) -> None:
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position = [position]
        publisher.publish(msg)
        self.sleep(1)
    
    def suck(self):
        self.send_serial_cmd(CMD_SUCK)
        self.sleep(8)
    
    def open_dispenser(self):
        self.send_serial_cmd(CMD_OPEN)
        self.sleep(2)

    def close_dispenser(self):
        self.send_serial_cmd(CMD_CLOSE)
        self.sleep(2)
    
    def set_separator(self, position: int):
        self.send_serial_cmd(CMD_POSE[position])
        self.sleep(2)

    def send_serial_cmd(self, cmd: int):
        print(hex(cmd))
        msg = bytes(chr(cmd), encoding="utf-8")
        self._serial.write(msg)
    
    def raise_manip(self):
        rospy.logwarn("Please raise manip")

    def lower_manip(self):
        rospy.logwarn("Please lower manip")
    
    def sleep(self, time: float):
        rospy.sleep(time)
    
class User:
    def __init__(self):
        self.trigger = False
        self.value = False
        rospy.Subscriber("/user_input", Bool, self.callback)
    
    def wait_for_user_ack(self):
        while not self.trigger:
            if rospy.is_shutdown():
                exit(1)
        self.trigger = False
        return self.value

    def callback(self, msg: Bool):
        self.trigger = True
        self.value = msg.data


class Node:
    def __init__(self):
        rospy.init_node("science")
        self.section_offset = 0
        self.hardware = ScienceHardware()
        self.user = User()

    def run(self, sample_index) -> None:
        rospy.loginfo(f"Sample {sample_index+1}")
        self.set_section(sample_index)
        self.set_carousel(SAMPLE)
        self.sample()
        self.set_carousel(INJECT)
        self.inject()
        self.set_carousel(MIX)
        self.mix()
        self.set_carousel(LITMUS)
        self.litmus(sample_index)
        self.set_carousel(INSPECT)
        self.inspect()
        rospy.loginfo(f"Thank you. Good luck with the next samples!")

    def sample(self) -> None:
        for flask_index in range(FLASK_PER_SAMPLE):
            self.set_separator(flask_index)
            rospy.loginfo(f"Filling flask {flask_index+1}/{FLASK_PER_SAMPLE}...")
            self.collect_flask()

    def collect_flask(self):
        flask_accepted = False
        self.close_dispenser()
        while not flask_accepted:
            self.suck()
            self.raise_manip()
            self.unclog()
            flask_accepted = self.wait_for_user_ack()
            self.lower_manip()
        self.open_dispenser()

    def unclog(self) -> None:
        self.suck()
        rospy.loginfo(f"Suction duct unclogged.")
    
    def mix(self) -> None:
        rospy.loginfo(f"Mixing...")
        for _ in range(MIXING_PASSES):
            self.increment_carousel(MIXING_OFFSET)
            self.increment_carousel(-2*MIXING_OFFSET)
            self.increment_carousel(MIXING_OFFSET)
    
    def increment_carousel(self, increment: float) -> None:
        self.set_carousel(self.hardware.get_carousel_position() + increment)

    def set_section(self, index) -> None:
        rospy.loginfo(f"Setting current section to {index+1}...")
        self.section_offset = index * SECTION_SIZE

    def set_carousel(self, position) -> None:
        self.set_carousel_absolute(position + self.section_offset)
    
    def set_carousel_absolute(self, position: float) -> None:
        rospy.logdebug(f"Carousel position set: {position}")
        self.hardware.set_carousel_position(position)

    def suck(self) -> None:
        rospy.loginfo(f"Sucking in progress...")
        self.hardware.suck()

    def raise_manip(self) -> None:
        rospy.loginfo(f"Raising manipulator...")
        self.hardware.raise_manip()
        self.user.wait_for_user_ack()
    
    def lower_manip(self) -> None:
        rospy.loginfo(f"Lowering manipulator...")
        self.hardware.lower_manip()
        self.user.wait_for_user_ack()

    def open_dispenser(self) -> None:
        rospy.loginfo(f"Dispenser opened.")
        self.hardware.open_dispenser()
    
    def close_dispenser(self) -> None:
        rospy.loginfo(f"Dispenser closed.")
        self.hardware.close_dispenser()

    def set_separator(self, position: int) -> None:
        rospy.loginfo(f"Separator set to position {position+1}")
        self.hardware.set_separator(position)

    def inject(self) -> None:
        rospy.loginfo(f"Injecting chemicals...")
        self.hardware.set_injector_position(self.hardware.get_injector_position() + INJECTOR_ROTATION)

    def litmus(self, position: float) -> None:
        rospy.loginfo(f"Inspect pH indicators.")
        self.increment_carousel(position * LITMUS_OFFSET)
        self.wait_for_user_ack()

    def inspect(self) -> None:
        rospy.loginfo(f"Done. Observe the reaction.")

    def wait_for_user_ack(self) -> bool:
        rospy.loginfo("Waiting for your interaction...")
        return self.user.wait_for_user_ack()
    
    def test(self):
        self.suck()
        self.open_dispenser()
        self.close_dispenser()
        self.set_separator(0)
        self.set_separator(1)
        self.set_separator(2)
        self.set_separator(3)
        



if __name__=="__main__":
    Node().run(0)