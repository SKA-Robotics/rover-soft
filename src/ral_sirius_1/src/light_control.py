import rospy
import serial
from std_msgs.msg import Int32
class DeviceWithoutParameter:
    def __init__(self, name):
        self.name = name
    def getting_parameter(self):
        param_name = rospy.search_param('arduino') #parameter to change 
        light_parameter = rospy.get_param (param_name)
        self.name = serial.Serial(light_parameter, 9600)    
    def light_on(self):
        try:
            self.name.write(ord("1"))
        except KeyError:
            print("Error occured")
            pass
class SerialDevice:
    def __init__(self, parameter, name):
        self.parameter = parameter
        self.name = name
    def set_parameter(self):
        self.name=serial.Serial(self.parameter, 9600)   
    def send_byte(self):
        self.name.write(ord("1"))
    def callback(self, data):
        self.name.write(data.encode())
def main():
    rospy.init_node("light_control") 
    device = SerialDevice("/dev/ttyUSB1", "A")
    starting_light = SerialDevice("/dev/ttyUSB2", "B")
    try:
        device.set_parameter()
        starting_light.set_parameter()
    except KeyError:
        print("Parameter is not set")
        pass
    starting_light.send_byte()
    rospy.Subscriber("status_light", Int32, device.callback)
    rospy.spin()
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass 
