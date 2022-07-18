#!/usr/bin/python3
import rospy
import serial
from std_msgs.msg import Int16

UART_PREFIX = 128
UART_SUFFIX = 192

MOISTURE_TOPIC = '/sensor_moisture'


class MoistureSensor:

    def __init__(self):
        rospy.init_node('moisture_sensor', log_level=rospy.DEBUG)
        self.moisture_pub = rospy.Publisher(MOISTURE_TOPIC, Int16, queue_size=10)

        self.port_name = rospy.get_param('~device', default='/dev/ttyACM2')
        self.baudrate = rospy.get_param('~baudrate', default=9600)
        try:
            self.serial_port = serial.Serial(self.port_name, self.baudrate)
        except serial.SerialException as e:
            rospy.logerr(f'Failed to open serial port: {self.port_name}')
            raise

    def __del__(self):
        self.serial_port.close()

    def run(self):
        while not rospy.is_shutdown():
            # Read data from sensors
            data = str(self.serial_port.readline())

            # Parse result for moisture percentage
            moisture_str = ""
            i = data.find('M') + 1
            while data[i] != "%":
                moisture_str += data[i]
                i += 1
            self.moisture_pub.publish(int(moisture_str))
            


if __name__ == '__main__':
    try:
        MoistureSensor().run()
    except rospy.ROSInterruptException:
        pass
