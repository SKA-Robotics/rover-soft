import rospy
import serial

def main():
    rospy.init_node("light_control") 
    A=serial.Serial("/dev/ttyUSB1", 9600)   
    A.write(1)
    rospy.spin()
   
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass 