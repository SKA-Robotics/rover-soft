import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped


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
        return filter_jointstate(self.jointstate, self.LINKS_DATA["names"])

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


class PosePublishingDecorator(ManipInterface):

    def __init__(self, interface: ManipInterface):
        self._component = interface
        self.ik_solver = None
        self.pose_publisher = None

    def set_ik_solver(self, ik_solver):
        self.ik_solver = ik_solver

    def set_topic_name(self, topic_name):
        self.pose_publisher = ROSPosePublisher(topic_name)

    def get_jointstate(self):
        return self._component.get_jointstate()

    def set_jointstate(self, jointstate: JointState):
        if self._can_publish():
            self._publish(jointstate)
        return self._component.set_jointstate(jointstate)

    def _publish(self, jointstate):
        pose = self.ik_solver.get_FK_solution(jointstate)
        self.pose_publisher.publish(pose)

    def _can_publish(self):
        return self.ik_solver is not None and self.pose_publisher is not None

    def get_manip_params(self):
        return self._component.get_manip_params()


class ROSPosePublisher:

    def __init__(self, topic):
        self.publisher = rospy.Publisher(topic, PointStamped, queue_size=10)

    def publish(self, pose):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.point.x = pose.x
        msg.point.y = pose.y
        msg.point.z = pose.z
        self.publisher.publish(msg)


def filter_jointstate(jointstate: JointState, names):
    filtered_jointstate = JointState()
    filtered_jointstate.header = jointstate.header
    for name in names:
        index = jointstate.name.index(name)
        filtered_jointstate.name.append(name)
        filtered_jointstate.position.append(jointstate.position[index])
        filtered_jointstate.velocity.append(jointstate.velocity[index])
        filtered_jointstate.effort.append(jointstate.effort[index])
    return filtered_jointstate
