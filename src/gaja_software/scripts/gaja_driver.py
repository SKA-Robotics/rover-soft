#!/usr/bin/python3
import serial
import rospy
from geometry_msgs.msg import Twist

class GajaDriver:
    def __init__(self, portname, baudrate):
        rospy.init_node('gaja_driver')
        rospy.loginfo('gaja_driver started.')
        self.sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        try:
            self.serial = serial.Serial(portname, baudrate)
        except serial.SerialException as e:
            rospy.logerr("Could not open serial port %s: %s" % (portname, e))
            exit(1)
        self.twist = Twist()
        self.last_command_time = rospy.Time.now()
        self.command_timeout = rospy.Duration(
            rospy.get_param('~command_timeout', 0.5))

        # Load kinematic description of the robot
        self.l_x = rospy.get_param('~l_x', 0.13)
        self.l_y = rospy.get_param('~l_y', 0.08)
        self.r = rospy.get_param('~r', 0.045)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            wheel_cmd = self.calculate_command()
            self.send_command(wheel_cmd)
            rate.sleep()
    
    def cmd_vel_callback(self, msg):
        # Stores the containts of command from /cmd_vel
        self.twist.linear.x = msg.linear.x
        self.twist.linear.y = msg.linear.y
        self.twist.angular.z = msg.angular.z
        self.last_command_time = rospy.Time.now()
    
    def calculate_command(self):

        # Stops the robot if no messages have been received
        if rospy.Time.now() - self.last_command_time > self.command_timeout:
            self.twist = Twist()
            rospy.logwarn("Command timeout")
        
        # Calculates wheels' velocities based on the command from /cmd_vel
        l_x = 0.13
        l_y = 0.08
        r = 0.045

        v_x = self.twist.linear.x
        v_y = self.twist.linear.y
        v_theta = self.twist.angular.z

        v_l = (l_x + l_y) * v_theta

        front_left = (v_x - v_y - v_l) / r
        front_right = (v_x + v_y + v_l) / r
        rear_left = (v_x + v_y - v_l) / r
        rear_right = (v_x - v_y + v_l) / r

        cmd = {
            'front_left': clamp(int(127 * front_left), -127, 127),
            'front_right': clamp(int(127 * front_right), -127, 127),
            'rear_left': clamp(int(127 * rear_left), -127, 127),
            'rear_right': clamp(int(127 * rear_right), -127, 127),
        }
        return cmd

    def send_command(self, msg):
        msg = " 128 0 %d 192 \n 128 1 %d 192 \n 128 2 %d 192 \n 128 3 %d 192" \
            % (msg['front_left'], msg['front_right'], msg['rear_left'], msg['rear_right'])
        rospy.logdebug("Sending via serial:\n"+msg)
        self.serial.write(msg.encode('utf-8'))

    def __del__(self):
        self.serial.close()

def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))

if __name__ == '__main__':
    try:
        driver = GajaDriver(
            rospy.get_param("~port_name", "/dev/ttyUSB0"),
            rospy.get_param("~baudrate", 9600))
        driver.run()
    except rospy.ROSInterruptException:
        pass