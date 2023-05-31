#!/usr/bin/python3
import rospy
from sirius_msgs.msg import FlagState
import can
can.rc['interface'] = 'socketcan'
can.rc['channel'] = 'slcan0'
can.rc['bitrate'] = 500000
from can.interface import Bus

ARBITRATION_ID = 69

bus = Bus()

def bits_to_int(bits):
    return int(''.join(map(str, map(int, bits))), 2)

def build_message(data):
    return can.Message(arbitration_id=ARBITRATION_ID,
                    is_extended_id=False,
                    data=[data])

def send_to_bus(message: FlagState):
    msg = build_message(bits_to_int([
            message.red,
            message.green,
            message.blue,
            message.yellow,
            message.buzzer
    ]))
    bus.send(msg)

rospy.init_node("led_flag")
rospy.Subscriber("~state", FlagState, send_to_bus, queue_size=10)
rospy.spin()
