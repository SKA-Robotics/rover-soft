#!/bin/python3
import rospy
import serial
from sirius_msgs.msg import JointState
from sensor_msgs.msg import JointState as SensorJointState

CMD_MANIP_TOPIC = '/cmd_manip'

LIMB_IDS = {
    'arm_rotate': "/roboclaw_driver/130/motor1/set_joint_state",
    'claw_clamp': "/roboclaw_driver/130/motor2/set_joint_state",
    'claw_rotate': "/roboclaw_driver/131/motor1/set_joint_state",
    'claw_lift': "/roboclaw_driver/131/motor2/set_joint_state",
    'arm_tilt': "/roboclaw_driver/132/motor1/set_joint_state",
    'arm_lift': "/roboclaw_driver/132/motor2/set_joint_state",
}


class ManipulatorController:

    def __init__(self):
        rospy.init_node('manip_control', log_level=rospy.DEBUG)

        rospy.Subscriber(CMD_MANIP_TOPIC, SensorJointState, self.cmd_manip_callback, queue_size=1, buff_size=2**24)

        self.publishers = {
            'arm_rotate': rospy.Publisher(LIMB_IDS['arm_rotate'], JointState, queue_size=10),
            'claw_clamp': rospy.Publisher(LIMB_IDS['claw_clamp'], JointState, queue_size=10),
            'claw_rotate': rospy.Publisher(LIMB_IDS['claw_rotate'], JointState, queue_size=10),
            'claw_lift': rospy.Publisher(LIMB_IDS['claw_lift'], JointState, queue_size=10),
            'arm_tilt': rospy.Publisher(LIMB_IDS['arm_tilt'], JointState, queue_size=10),
            'arm_lift': rospy.Publisher(LIMB_IDS['arm_lift'], JointState, queue_size=10),
        }

        self.latest_msg = None
        self.rate = 10

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.latest_msg is not None:
                self.publish_latest_msg()
            rate.sleep()

    def cmd_manip_callback(self, msg: JointState):
        self.latest_msg = msg

    def publish_latest_msg(self):
        for i, joint_name in enumerate(self.latest_msg.name):
            try:
                #limb_id = LIMB_IDS[joint_name]
                message = JointState(velocity=[],
                                     position=[],
                                     acceleration=[],
                                     effort=[],
                                     duty=[self.latest_msg.effort[i] * 100])
                message.header.stamp = rospy.get_rostime()
                self.publishers[joint_name].publish(message)
            except KeyError:
                rospy.logerr(f'{CMD_MANIP_TOPIC} received invalid limb \'{joint_name}\'')


if __name__ == '__main__':
    try:
        ManipulatorController().run()
    except rospy.ROSInterruptException:
        pass
