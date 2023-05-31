#!/usr/bin/python3
import rospy
from sirius_msgs.msg import FlagState
import can
can.rc['interface'] = 'socketcan'
can.rc['channel'] = 'slcan0'
can.rc['bitrate'] = 500000
from can.interface import Bus

ARBITRATION_ID = 0x69
STRING_LENGTH_LIMIT = 32
STRING_ARBITRATION_IDS = [0x6a, 0x6b, 0x6c, 0x6d]

bus = Bus()

def bits_to_int(bits):
    return int(''.join(map(str, map(int, bits))), 2)

def build_flag_message(data):
    return can.Message(arbitration_id=ARBITRATION_ID,
                    is_extended_id=False,
                    data=[data])

def string_to_bytes(data):
    return [ord(character) for character in data]

def string_to_bytelists(data):
    if len(data) > STRING_LENGTH_LIMIT:
        rospy.logwarn(f"Message too long! {len(data)}/{STRING_LENGTH_LIMIT} characters")
    data = data[:STRING_LENGTH_LIMIT].ljust(STRING_LENGTH_LIMIT, '\x00')
    return (
        string_to_bytes(data[0:8]),
        string_to_bytes(data[8:16]),
        string_to_bytes(data[16:24]),
        string_to_bytes(data[24:32]),
    )
    
def build_string_messages(data):
    bytelists = string_to_bytelists(data)
    return [
        can.Message(arbitration_id=arbitration_id, is_extended_id=False, data=bytes)
        for bytes, arbitration_id in zip(bytelists, STRING_ARBITRATION_IDS)]

def send_to_bus(message: FlagState):
    msg = build_flag_message(bits_to_int([
            message.red,
            message.green,
            message.blue,
            message.yellow,
            message.buzzer
    ]))
    print(msg)
    bus.send(msg)
    string_msgs = build_string_messages(message.message)
    for string_msg in string_msgs:
        rospy.sleep(rospy.Duration(0.1))
        bus.send(string_msg)


rospy.init_node("led_flag")
rospy.Subscriber("~state", FlagState, send_to_bus, queue_size=10)
rospy.spin()
