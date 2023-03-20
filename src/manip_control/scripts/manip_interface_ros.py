import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped

from manip_interface import ManipInterface, ManipParams
from ik import ManipJointState


class ROSManipInterface(ManipInterface):

    def __init__(self):
        super().__init__()
        rospy.loginfo("Initializg ROSManipInterface")
        self.jointstate = None
        self._params = self._load_ROSparams()
        self._initialize_subscriber()
        self._initialize_publishers()

    def _initialize_subscriber(self):
        self.jointstate_topic = rospy.get_param("~jointstate_topic", "/manipulator/joint_states")
        self.command_topics = rospy.get_param("~command_topics", {})
        self.subscriber = rospy.Subscriber(self.jointstate_topic, JointState, self._update_jointstate)

    def _load_ROSparams(self):
        mode_params = rospy.get_param("~control_modes")
        link_params = rospy.get_param("~links")
        params_dict = {"links": link_params, "control_modes": mode_params}
        return ManipParams.from_dict(params_dict)
    
    def get_jointstate(self) -> ManipJointState:
        return self.jointstate

    def set_jointstate(self, jointstate: ManipJointState):
        self.jointstate = jointstate
        jointstate_cmd = JointStateConverter.manip_to_ros(jointstate, self._params.joint_names())
        self._distribute_joint_commands(jointstate_cmd)

    def _update_jointstate(self, msg):
        self.jointstate = JointStateConverter.ros_to_manip(msg, self._params.joint_names())
        self.subscriber.unregister()

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

    def sleep(self, time):
        rospy.sleep(rospy.Duration(time))

    def get_manip_params(self) -> ManipParams:
        return self._params


class PosePublishingDecorator(ManipInterface):

    def __init__(self, interface: ManipInterface):
        self._component = interface
        self.ik_solver = None
        self.pose_publisher = None

    def set_ik_solver(self, ik_solver):
        self.ik_solver = ik_solver

    def set_topic_name(self, topic_name):
        self.pose_publisher = ROSPosePublisher(topic_name)

    def get_jointstate(self) -> ManipJointState:
        return self._component.get_jointstate()

    def set_jointstate(self, jointstate: ManipJointState):
        if self._can_publish():
            self._publish(jointstate)
        return self._component.set_jointstate(jointstate)

    def _publish(self, jointstate: ManipJointState):
        pose = self.ik_solver.get_FK_solution(jointstate)
        self.pose_publisher.publish(pose)

    def _can_publish(self):
        return self.ik_solver is not None and self.pose_publisher is not None

    def sleep(self, time):
        return self._component.sleep(time)

    def get_manip_params(self):
        return self._component.get_manip_params()


class JointstatePublishingDecorator(ManipInterface):

    def __init__(self, interface: ManipInterface):
        self._component = interface
        self.jointstate_publisher = None

    def set_topic_name(self, topic_name):
        self.jointstate_publisher = ROSJointStatePublisher(topic_name)

    def get_jointstate(self) -> ManipJointState:
        return self._component.get_jointstate()

    def set_jointstate(self, jointstate: ManipJointState):
        if self._can_publish():
            self._publish(jointstate)
        return self._component.set_jointstate(jointstate)

    def _publish(self, jointstate: ManipJointState):
        ros_jointstate = JointStateConverter.manip_to_ros(jointstate, self._component.get_manip_params().joint_names())
        self.jointstate_publisher.publish(ros_jointstate)

    def _can_publish(self):
        return self.jointstate_publisher is not None

    def sleep(self, time):
        return self._component.sleep(time)

    def get_manip_params(self):
        return self._component.get_manip_params()


class ROSJointStatePublisher:

    def __init__(self, topic):
        self.publisher = rospy.Publisher(topic, JointState, queue_size=10)

    def publish(self, jointstate):
        jointstate.header.stamp = rospy.Time.now()
        self.publisher.publish(jointstate)


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


class JointStateConverter:

    def manip_to_ros(joinstate: ManipJointState, names) -> JointState:
        assert len(joinstate.position) == len(names)
        result = JointState()
        result.header.stamp = rospy.Time.now()
        result.name = names
        result.position = joinstate.position
        return result

    def ros_to_manip(jointstate: JointState, names) -> ManipJointState:
        filtered_jointstate = JointStateConverter.filter_ros(jointstate, names)
        return ManipJointState.from_list(filtered_jointstate.position)

    def filter_ros(jointstate: JointState, names):
        filtered_jointstate = JointState()
        filtered_jointstate.header = jointstate.header
        for name in names:
            index = jointstate.name.index(name)
            filtered_jointstate.name.append(name)
            filtered_jointstate.position.append(jointstate.position[index])
            filtered_jointstate.velocity.append(jointstate.velocity[index])
            filtered_jointstate.effort.append(jointstate.effort[index])
        return filtered_jointstate
