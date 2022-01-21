import rospy
import serial
from std_msgs.msg import Int32

def callback(data):
     A=serial.Serial("/dev/ttyUSB1", 9600)
     A.write(data.encode())
def main():
    rospy.init_node("light_control") 
    #A=serial.Serial("/dev/ttyUSB1", 9600)   #parametr powinien byc wczytywany
    rospy.Subscriber("status_light", Int32, callback)
    rospy.spin()
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass 
#przerobic na obiektowe
#serial rozpisac  by miec dostep wszedzie
#nie inicjowac urzadzenia szeregowego za kazda utrzymana informacje