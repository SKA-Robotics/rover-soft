#!/usr/bin/python3
import rospy
import can_utils as can
from std_msgs.msg import Float64

ARBITRATION_ID = 0x42
CAN_SEND_TOPIC = "/sent_messages"
PUBLISH_RATE = 10
GRIPPER_SPEED = 25
GRIPPER_MIN = 0
GRIPPER_START = 100
GRIPPER_MAX = 255


class Node():
    def __init__(self):
        rospy.init_node("cripper")
        self._gripper_position = GRIPPER_START
        self._publisher = rospy.Publisher(CAN_SEND_TOPIC, can.Frame, queue_size=10)
        self._rate = rospy.Rate(PUBLISH_RATE)
        rospy.Subscriber("/cmd_grip", Float64, self.update_gripper_command, queue_size=10)
    
    def run(self):
        while not rospy.is_shutdown():
            self.send_command_to_bus()
            self._rate.sleep()

    def update_gripper_command(self, msg):
        self._gripper_position += msg.data * GRIPPER_SPEED
        self._gripper_position = min(GRIPPER_MAX, max(GRIPPER_MIN, self._gripper_position))

    def send_command_to_bus(self):
        m = can.make_message(ARBITRATION_ID, [int(self._gripper_position)])
        self._publisher.publish(m)

if __name__ == "__main__":
    Node().run()