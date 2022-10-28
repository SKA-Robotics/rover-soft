#!/usr/bin/python3
import rospy
import serial
from serial.tools import list_ports
from sirius_msgs.msg import JointState

MOTOR1_TOPIC = "/roboclaw_driver/132/motor1/set_joint_state",
MOTOR2_TOPIC = "/roboclaw_driver/132/motor2/set_joint_state",


class ManipArduino:

    def __init__(self):
        rospy.init_node('manip_arduino', log_level=rospy.DEBUG)
        self.motor1_cmd = 0
        self.motor2_cmd = 0
        self.sub1 = rospy.Subscriber("/roboclaw_driver/130/motor1/set_joint_state",
                                     JointState,
                                     self.callback_motor1,
                                     queue_size=10)
        self.sub2 = rospy.Subscriber("/roboclaw_driver/130/motor2/set_joint_state",
                                     JointState,
                                     self.callback_motor2,
                                     queue_size=10)
        self.port_name = rospy.get_param('~device', default='/dev/ttyACM0')
        self.baudrate = rospy.get_param('~baudrate', default=9600)

        ports = list_ports.comports()
        print(ports[0].hwid)
        for port, desc, hwid in sorted(ports):
            if hwid.lower().find("1a86") >= 0:
                print(port)
                print(desc)
                self.port_name = port
        try:
            self.serial_port = serial.Serial(self.port_name, self.baudrate, write_timeout=1)
        except serial.SerialException as e:
            rospy.logerr(f'Failed to open serial port: {self.port_name}')
            raise

    def __del__(self):
        self.serial_port.close()

    def callback_motor1(self, msg: JointState):
        self.motor1_cmd = msg.duty[0]

    def callback_motor2(self, msg: JointState):
        self.motor2_cmd = msg.duty[0]

    def write_serial(self):
        direction1 = (1 if self.motor1_cmd > 0 else 0)
        direction2 = (1 if self.motor2_cmd > 0 else 0)
        speed1 = bin(int(abs(self.motor1_cmd) * 0.07))[2:].rjust(3, '0')
        speed2 = bin(int(abs(self.motor2_cmd) * 0.07))[2:].rjust(3, '0')
        msg = str(direction1) + speed1 + str(direction2) + speed2
        try:
            self.serial_port.write(int(msg, 2).to_bytes(1, 'big'))
            print(int(msg, 2).to_bytes(1, 'big'), "\t", msg)
        except serial.serialutil.SerialTimeoutException:
            rospy.logwarn("Serial write timeout")

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.write_serial()
            r.sleep()


if __name__ == "__main__":
    try:
        ManipArduino().run()
    except rospy.ROSInterruptException:
        pass
