#!/usr/bin/python3
import rospy
from enum import Enum
from queue import Queue

from geometry_msgs.msg import PointStamped, Twist
from std_srvs.srv import Empty, EmptyResponse

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
        self.mode = mode.CARTESIAN
        self.joystick_receiver = JoystickReceiver("/cmd_manip", 0.5)
        self.rate = rospy.Rate(20)
        self.pending_moves = Queue(64)

        rospy.Subscriber("/clicked_point", PointStamped, self.callback, queue_size=10)
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
            move = self.pending_moves.get()
            move[0](move[1])
            rospy.loginfo("Target pose reached")
            
    def _execute_joystick_command(self):
        deltatime = self.rate.sleep_dur.to_sec()
        pose_delta = self.joystick_receiver.get_pose_delta(deltatime)
        self.manip.move_incremental(pose_delta)

    def callback(self, data):
        rospy.loginfo("Received request:\n" + str(data.point))
        target = ManipPose()
        target.x = data.point.x
        target.y = data.point.y
        target.z = data.point.z
        if self.mode == mode.CARTESIAN:
            self.pending_moves.put((self.manip.move_cartesian, target))
        else:
            self.pending_moves.put((self.manip.move_jointspace, target))
    
    def _handle_toggle_mode(self, req):
        self._toggle_mode()
        rospy.loginfo(f"Switched to {self.mode.name} mode")
        return EmptyResponse()
    
    def _toggle_mode(self):
        if self.mode == mode.CARTESIAN:
            self.mode = mode.JOINTSPACE
        else:
            self.mode = mode.CARTESIAN


class JoystickReceiver:

    def __init__(self, joystick_topic, timeout):
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

    def _is_timed_out(self):
        result = rospy.Time.now() - self.prev_time > rospy.Duration(self._timeout)
        self.prev_time = rospy.Time.now()
        return result

    def _twist_to_pose_scaled(self, twist: Twist, scale):
        values = [twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z]
        values = [x * scale for x in values]
        return ManipPose.from_list(values)


if __name__ == "__main__":
    ManipController().run()
