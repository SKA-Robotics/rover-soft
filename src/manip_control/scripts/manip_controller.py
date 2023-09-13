#!/usr/bin/python3
import rospy
from enum import Enum
from queue import Queue

from geometry_msgs.msg import PointStamped, Twist
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest

from manip import SiriusManip
from manip_interface_ros import ROSManipInterface
from ik import ManipPose


class mode(Enum):
    CARTESIAN = 0
    JOINTSPACE = 1


class ManipController:

    def __init__(self):
        rospy.init_node("manip_controller")
        interface = ROSManipInterface()
        self.manip = SiriusManip(interface)
        self.mode = mode.JOINTSPACE
        joystick_timeout: float = rospy.get_param("~joystick_timeout", 0.5)
        self.joystick_receiver = JoystickReceiver("/cmd_manip", joystick_timeout)
        self.rate = rospy.Rate(rospy.get_param("~control_modes/incremental/send_rate"))
        self.pending_moves = Queue(rospy.get_param("~queue_size", 16))

        rospy.Subscriber("/cmd_manip_pos", PointStamped, self.callback, queue_size=10)
        rospy.Service("~toggle_mode", Empty, self._handle_toggle_mode)

    def run(self):
        while not rospy.is_shutdown():
            try:
                self._execute_pending_moves()
                self._execute_joystick_command()
                self.rate.sleep()
            except Exception as e:
                rospy.logwarn(e)

    def _execute_pending_moves(self):
        while not self.pending_moves.empty():
            move: 'tuple[function, ManipPose]' = self.pending_moves.get()
            move[0](move[1])
            rospy.loginfo("Target pose reached")

    def _execute_joystick_command(self):
        deltatime = self.rate.sleep_dur.to_sec()
        pose_delta = self.joystick_receiver.get_pose_delta(deltatime)
        self.manip.move_incremental(pose_delta)

    def callback(self, data: PointStamped):
        rospy.loginfo("Received request:\n" + str(data.point))
        target = ManipPose()
        target.x = data.point.x
        target.y = data.point.y
        target.z = data.point.z
        target.roll = 0.0
        target.pitch = 0.0
        target.yaw = 0.0
        if self.mode == mode.CARTESIAN:
            self._put_into_pending_moves((self.manip.move_cartesian, target))
        else:
            self._put_into_pending_moves((self.manip.move_jointspace, target))

    def _put_into_pending_moves(self, move: 'tuple[function, ManipPose]'):
        if self.pending_moves.full():
            rospy.logwarn("Move queue full. Dropping first element to insert new at the end.")
            self.pending_moves.get()
        self.pending_moves.put(move)

    def _handle_toggle_mode(self, req: EmptyRequest):
        self._toggle_mode()
        rospy.loginfo(f"Switched to {self.mode.name} mode")
        return EmptyResponse()

    def _toggle_mode(self):
        if self.mode == mode.CARTESIAN:
            self.mode = mode.JOINTSPACE
        else:
            self.mode = mode.CARTESIAN


class JoystickReceiver:

    def __init__(self, joystick_topic: str, timeout: float):
        self._velocity = Twist()
        self._timeout = timeout
        self.prev_time = rospy.Time.now()
        rospy.Subscriber(joystick_topic, Twist, self._set_velocity, queue_size=10)

    def _set_velocity(self, velocity: Twist):
        self._velocity = velocity

    def get_pose_delta(self, deltatime):
        if self._is_timed_out():
            self._velocity = Twist()
        return self._twist_to_pose_scaled(self._velocity, deltatime)

    def _is_timed_out(self) -> bool:
        result = rospy.Time.now() - self.prev_time > rospy.Duration(self._timeout)
        self.prev_time = rospy.Time.now()
        return result

    def _twist_to_pose_scaled(self, twist: Twist, scale: float):
        values = [twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z]
        values = [x * scale for x in values]
        return ManipPose.from_list(values)


if __name__ == "__main__":
    ManipController().run()
