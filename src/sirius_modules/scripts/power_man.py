#!/usr/bin/python3
import rospy
from sirius_msgs.msg import BatteryState
import serial

PUBLISH_PERIOD = 15
PORT_NAME = "/dev/ttyACM21"
BAUDRATE = 115200

class Node:
    def __init__(self):
        rospy.init_node("power_man")
        self._publisher = rospy.Publisher("~battery_state", BatteryState, queue_size=20)
        self._message = BatteryState()
        self._serial_port = serial.Serial(PORT_NAME, BAUDRATE)
        rospy.Timer(rospy.Duration(PUBLISH_PERIOD), self.publish)
    
    def publish(self, _):
        self._update_message_header()
        self._publisher.publish(self._message)

    def run(self):
        while not rospy.is_shutdown():
            data = str(self._serial_port.readline())[1:]
            if (data[1:3] != "im"): # message not complete
                break
            voltage, i_motor, i_pc, i_can = self._parse_string(data)
            self._message.voltage = voltage
            self._message.motor_current = i_motor
            self._message.computer_current = i_pc
            self._message.can_current = i_can
    
    def _parse_string(self, data):
        return (
            self._find_value(data, "vv"),
            self._find_value(data, "im"),
            self._find_value(data, "ip"),
            self._find_value(data, "ic"))

    def _find_value(self, data, key):
        value_str = ""
        i = data.find(key) + 2 
        while data[i] != ";":
            value_str += data[i]
            i += 1
        return float(value_str)          
    
    def _update_message_header(self):
        self._message.header.stamp = rospy.Time.now()

if __name__=="__main__":
    Node().run()