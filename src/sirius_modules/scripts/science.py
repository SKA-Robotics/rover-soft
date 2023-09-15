import rospy
from sirius_msgs.msg import JointState as SiriusJointState
from sensor_msgs.msg import JointState as JointState
from std_msgs.msg import Float32

DRILL_ROBOCLAW_TOPIC = "/roboclaw_driver/131/motor1/set_joint_state"
PUSH_ROBOCLAW_TOPIC = "/roboclaw_driver/132/motor2/set_joint_state"
LIFT_ROBOCLAW_TOPIC = "/roboclaw_driver/131/motor2/set_joint_state"

INTERFACE_TOPIC = "/science/joint_state"

drill_speed = 10.0
push_speed = 1.0
lift_speed = 3.0

timeout = rospy.Duration(1.0)
publish_rate = 10.0

def clampToUnitRange(value):
    return min(1.0, max(-1.0, value)) 

class Node:
    def __init__(self) -> None:
        rospy.init_node("science")
        self.drill_publisher = rospy.Publisher(DRILL_ROBOCLAW_TOPIC, SiriusJointState, queue_size=10)
        self.push_publisher = rospy.Publisher(PUSH_ROBOCLAW_TOPIC, SiriusJointState, queue_size=10)
        self.lift_publisher = rospy.Publisher(LIFT_ROBOCLAW_TOPIC, SiriusJointState, queue_size=10)
        rospy.Subscriber(INTERFACE_TOPIC, JointState, self.interface_callback, queue_size=10)
        self.drill_speed = 0.0
        self.push_speed = 0.0
        self.lift_speed = 0.0
        rospy.Timer(rospy.Duration.from_sec(1/publish_rate), self.publish_messages)
        self.last_command = rospy.Time.now()

    def run(self):
        rospy.spin()
    
    def interface_callback(self, msg):
        self.drill_speed = clampToUnitRange(msg.effort[0]) * drill_speed
        self.push_speed = clampToUnitRange(msg.effort[1]) * push_speed
        self.lift_speed = clampToUnitRange(msg.effort[2]) * lift_speed
        self.last_command = rospy.Time.now()

    def publish_messages(self, _):
        if (self.last_command + timeout < rospy.Time.now()):
            rospy.logwarn("Command timeout")
            self.stop()
        self.send_effort_command(self.drill_publisher, self.drill_speed)
        self.send_effort_command(self.push_publisher, self.push_speed)
        self.send_effort_command(self.lift_publisher, self.lift_speed)

    def stop(self):
        self.drill_speed = 0.0
        self.push_speed = 0.0
        self.lift_speed = 0.0

    def send_effort_command(self, publisher : rospy.Publisher, effort : float):
        msg = SiriusJointState()
        msg.header.stamp = rospy.Time.now()
        msg.effort = [effort]
        publisher.publish(msg)






if __name__=="__main__":
    print("Goodbye world!")
    node = Node()
    node.run()
