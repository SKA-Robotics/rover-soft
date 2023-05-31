#!/usr/bin/python3
import rospy
from sirius_msgs.msg import BatteryState

PUBLISH_RATE = 0.3

class Node:
    def __init__(self):
        rospy.init_node("power_man")
        self._publisher = rospy.Publisher("~battery_state", BatteryState, queue_size=20)
        self._message = BatteryState()
        self._rate = rospy.Rate(PUBLISH_RATE)
    
    def run(self):
        while not rospy.is_shutdown():
            self._update_message_header()
            self._publisher.publish(self._message)
            self._rate.sleep()
    
    def _update_message_header(self):
        self._message.header.stamp = rospy.Time.now()

if __name__=="__main__":
    Node().run()