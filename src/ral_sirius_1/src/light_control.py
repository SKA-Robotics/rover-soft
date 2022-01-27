import rospy
import serial
from std_msgs.msg import Int32
class SerialDevice:
    def __init__(self, parameter, name):
        self.parameter = parameter
        self.name = name
    def set_parameter(self):
       self.name=serial.Serial(self.parameter, 9600)
    def callback(self, data):
        self.name.write(data.encode())
def main():
    rospy.init_node("light_control") 
    device = SerialDevice("/dev/ttyUSB1", "A")
    try:
        device.set_parameter()
    except KeyError:
        print("Parameter is not set")
        pass
    rospy.Subscriber("status_light", Int32, device.callback)
    rospy.spin()
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass 
