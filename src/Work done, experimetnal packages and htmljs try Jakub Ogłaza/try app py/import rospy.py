import rospy
from geometry_msgs.msg import Twist
import sys
#inicjalizujemy funkcje twist po zaimportowaniu wszystkich potrzebnych paczek
twist = Twist()
#definiujemy i czytamy z klawiatury przyciski i dodajemy do nich ruck, wsad to nasze przyciski
def values():
    print ('w do przodu, a w lewo, s w tył, d w prawo, q zatrzymuje pojazd\n')
    s = raw_input(':- ')
    if s[0] == 'w':
        twist.linear.x = 1.0
        twist.linear.y = 0.0
        print ('wcisnieto w\n')

    elif s[0] == 's':
        twist.linear.x = -1.0
        twist.linear.y = 0.0
        print ('wcisnieto s\n')
    elif s[0] == 'd':
        twist.linear.y = -1.0
        twist.linear.x = 0.0
        print ('wcisnieto d\n')
    elif s[0] == 'a':
        twist.linear.y = 1.0
        twist.linear.x = 0.0
        print ('wcisnieto a\n')
    elif s[0] == 'q':
        twist.angular.z = twist.linear.x = twist.linear.y = 0.0
        print ('hamulec uzyty\n')
        sys.exit()
    else:
        twist.linear.x = twist.linear.y = twist.angular.z = 0.0
        print ('Ten przycik nie powoduje żadnej reakcji \n')
    return twist
#definicja funkcji klawiatury i łaczenie jej do rosa jako node
def klawiatura():
    pub = rospy.Publisher('base_controller/command',Twist, queue_size=1)
    rospy.init_node('teleop_py',anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():  #twist dziala gdy rospy dziala
        twist = values()
        pub.publish(twist)
        rate.sleep()
#dbamy by nikt nie uruchamial programu przez przypadek 
if __name__ == '__main__':
    try:
        klawiatura()
    except rospy.ROSInterruptException:   #i by nie wystapily bledy
        pass