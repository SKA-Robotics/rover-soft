import rospy #importy
import sys
import serial
from std.msgs import byte.msg
try:
    serial_parameterA=rospy.get_param('parameterA') #parametry lampek do wpisania
    serial_parameterB=rospy.get_param('parameterB') 
except KeyError:
    print("Parameter is not set")
    exit()
sendingByte=1
A = serial.Serial(serial_parameterA, 9600) 
B = serial.Serial(serial_parameterB, 9600)
def lighton():
      command = input()
      if command[0] == '1':
         A.write(command.encode())
      elif command[0] == '0':
         A.write(command.encode())  #potrzebny pomysl jak sobie poradzic gdy ktos wcisnie innny przycisk
#inicjacja urzadzenia szeregowego, fukcja wlacz swiatlo gdy przycisk wcisniety
def ledcontrol():
    rospy.init_node('led_control',anonymous=True)
    pub = rospy.Publisher('ral_sirius_1',byte.msg, queue_size=1) # znalezc typ dla publishera
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():  
        #jedno swiatlo tutaj od razu sie wlaczy gdy bedzie sie wlaczal node
        B.write(sendingByte.encode()) 
        lighton()
        pub.publish()
        rate.sleep()
if __name__ == '__main__':
    try:
        ledcontrol()
    except rospy.ROSInterruptException:  
        pass
#potrzebny sposob na inicjowanie z rosem node(robot upstart), wylaczanue swuatla inicjowanego z nodem,  ogarnac sposob komuikacji z urzadzenuiem szeregowym dokladniej
#pomyslec jakie bledy moga wystapic przy nierozwaznym uzywaniu
