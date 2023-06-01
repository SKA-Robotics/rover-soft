#!/usr/bin/python3
import rospy
from sirius_msgs.msg import FlagState
import can_utils as can

ARBITRATION_ID = 0x69
STRING_ARBITRATION_IDS = [0x6a, 0x6b, 0x6c]
CAN_SEND_TOPIC = "/sent_messages"

publisher = rospy.Publisher(CAN_SEND_TOPIC, can.Frame, queue_size=10)

def send_to_bus(msg: FlagState):
    bits = [msg.red, msg.green, msg.blue, msg.yellow, msg.buzzer]
    m = can.make_message(ARBITRATION_ID, [can.bits_to_int(bits)])
    publisher.publish(m)
    rospy.sleep(0.01)
    msgs = can.make_string_messages(STRING_ARBITRATION_IDS, msg.text)
    for m in msgs:
        publisher.publish(m)
        rospy.sleep(0.01)
    rospy.loginfo(
        f"Sending to LED Flag: rgby: {int(msg.red)}{int(msg.green)}{int(msg.blue)}{int(msg.yellow)},\
 buzzer: {('on' if msg.buzzer else 'off')}, text: \"{msg.text}\"")


rospy.init_node("led_flag")
rospy.Subscriber("~state", FlagState, send_to_bus, queue_size=10)
rospy.spin()

