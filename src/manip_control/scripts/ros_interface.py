import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class ManipInterface:

    def get_jointstate(self):
        pass

    def set_jointstate(self, jointstate: JointState):
        pass

    def get_manip_params(self):
        pass


class ROSManipInterface(ManipInterface):

    def __init__(self):
        super().__init__()
        rospy.loginfo("Initializg ROSManipInterface")
        self._load_ROSparams()
        self.jointstate_topic = rospy.get_param("~jointstate_topic", "/manipulator/joint_states")
        self.command_topics = rospy.get_param("~command_topics", {})
        self.jointstate = JointState()
        rospy.Subscriber(self.jointstate_topic, JointState, self._update_jointstate)
        self._initialize_publishers()

    def _load_ROSparams(self):
        self.MODES_DATA = rospy.get_param("~control_modes")
        self.LINKS_DATA = rospy.get_param("~links")

    def get_jointstate(self):
        return self.jointstate

    def set_jointstate(self, jointstate: JointState):
        self._distribute_joint_commands(jointstate)

    def _update_jointstate(self, msg):
        self.jointstate = msg

    def _initialize_publishers(self):
        self.publishers = {}
        info = "Initialized publishers:"
        for joint, topic in self.command_topics.items():
            self.publishers[joint] = rospy.Publisher(topic, Float64, queue_size=10)
            info = info + f"\n    - {joint}: {topic}"
        rospy.loginfo(info)

    def _distribute_joint_commands(self, jointstate: JointState):
        for index, value in enumerate(jointstate.position):
            publisher = self.publishers[jointstate.name[index]]
            publisher.publish(Float64(value))

    def get_manip_params(self):
        return {"control_modes": self.MODES_DATA, "links": self.LINKS_DATA}