#!/bin/python3
import rospy
import serial
from sensor_msgs.msg import JointState

UART_PREFIX = 128
UART_SUFFIX = 192

CMD_MANIP_TOPIC = '/cmd_manip'
DEFAULT_SPEED = 60

LIMB_IDS = {
    'arm_rotate': 0,
    'claw_clamp': 1,
    'claw_rotate': 2,
    'claw_lift': 3,
    'arm_tilt': 4,
    'arm_lift': 5,
}


class ManipulatorController:

    def __init__(self):
        rospy.init_node('manip_control', log_level=rospy.DEBUG)

        self.speeds = [DEFAULT_SPEED] * len(LIMB_IDS)
        for (limb_name, limb_id) in LIMB_IDS.items():
            self.speeds[limb_id] = rospy.get_param('~speeds/' + limb_name, default=DEFAULT_SPEED)

        self.port_name = rospy.get_param('~device', default='/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', default=9600)
        try:
            self.serial_port = serial.Serial(self.port_name, self.baudrate)
        except serial.SerialException as e:
            rospy.logerr(f'Failed to open serial port: {self.port_name}')
            raise

        rospy.Subscriber(CMD_MANIP_TOPIC, JointState, self.cmd_manip_callback, queue_size=1, buff_size=2**24)

    def __del__(self):
        self.serial_port.close()

    def run(self):
        rospy.spin()

    def cmd_manip_callback(self, msg: JointState):
        for i, joint_name in enumerate(msg.name):
            try:
                limb_id = LIMB_IDS[joint_name]
                effort = msg.effort[i] * self.speeds[limb_id]
                self.send_uart(limb_id, int(effort))
            except KeyError:
                rospy.logerr(f'{CMD_MANIP_TOPIC} received invalid limb \'{joint_name}\'')

    def send_uart(self, limb_id: int, value: int):
        data = ' '.join(str(x) for x in [UART_PREFIX, limb_id, value, UART_SUFFIX]) + '\n'
        rospy.logdebug(f'Sending to {self.port_name}: \'{data.strip()}\'')
        self.serial_port.write(bytes(data, 'utf8'))
        self.serial_port.flush()


if __name__ == '__main__':
    try:
        ManipulatorController().run()
    except rospy.ROSInterruptException:
        pass
