#!/usr/bin/python3
import rospy
import math
import random
import traceback
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from sirius_msgs.msg import ManipPose
from std_msgs.msg import Float64

from sirius_ik import IKSolver
from ros_interface import ManipInterface, ROSManipInterface


#TODO: decouple logic and ROS specific code.
class SiriusManip:

    def __init__(self, manip_interface : ManipInterface):
        self.manip_interface = manip_interface

        self.MODES_DATA = rospy.get_param("~control_modes")
        self.LINKS_DATA = rospy.get_param("~links")
        self.solver = IKSolver(self.LINKS_DATA["names"], self.LINKS_DATA["lengths"], self.LINKS_DATA["limits"])

        rospy.Subscriber("/clicked_point", PointStamped, self.point_callback, queue_size=10)
        self.point_publisher = rospy.Publisher("/received_point", PointStamped, queue_size=10)
        self.jointstate_publisher = rospy.Publisher("/joint_states", JointState, queue_size=10)

        # TODO read actual joint state, remove the line below when it's done
        self.currentPos = ManipPose()
        self.currentPos.x = 0.5
        self.currentPos.y = 0.0
        self.currentPos.z = 0.25
        self.currentPos.pitch = 0.0
        # This is temporary, to make it work for SAR. After encoders are install integate it properly.

    def point_callback(self, data):
        rospy.loginfo("Received request:\n" + str(data.point))
        target = ManipPose()
        target.x = data.point.x
        target.y = data.point.y
        target.z = data.point.z
        target.pitch = random.uniform(-math.pi / 4, 0)

        self.move_cartesian(target)

    def _move(self, pose: ManipPose):
        jointstate = JointState()
        try:
            jointstate = self.solver.get_IK_solution(pose)
            self._publish_jointstate(jointstate)
            # TODO read actual joint state, remove the line below when it's done
            self.currentPos = pose
        except Exception as e:
            #TODO add diagnostic information
            rospy.logwarn("Could not move to target point")
            rospy.logwarn(traceback.format_exc())
            return

        ik_fk = self.solver.get_FK_solution(jointstate)
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "root_link"
        msg.point.x = ik_fk.x
        msg.point.y = ik_fk.y
        msg.point.z = ik_fk.z
        self.point_publisher.publish(msg)

    def _publish_jointstate(self, jointstate: JointState):
        self.manip_interface.set_jointstate(jointstate)

    def get_pose(self) -> ManipPose:
        self.currentPos = self.solver.get_FK_solution(self.get_jointstate())
        return self.currentPos

    def get_jointstate(self) -> JointState:
        return self.manip_interface.get_jointstate()

    # For traversing small distances in a straight line. Using this
    # mode to span large distances may lead to errors as the manipulator
    # would enter collision with itself
    def move_cartesian(self, target_pose: ManipPose):
        pose = self.get_pose()

        # Calculate the vector of the movement
        direction = [
            target_pose.x - pose.x, target_pose.y - pose.y, target_pose.z - pose.z, target_pose.pitch - pose.pitch
        ]
        distance = math.sqrt(direction[0]**2 + direction[1]**2 + direction[2]**2 + direction[3]**2)
        if distance == 0:
            print("Target point is the same as current point")
            return
        direction = [x / distance for x in direction]

        # Load movement parameters
        acceleration = self.MODES_DATA["cartesian"]["acceleration"]
        max_velocity = self.MODES_DATA["cartesian"]["max_velocity"]
        error = self.MODES_DATA["cartesian"]["max_error"]
        rate = rospy.Rate(self.MODES_DATA["cartesian"]["interpolation_rate"])

        velocity = 0.0
        acceleration_distance = 0
        decceleration_distance = 0
        decceleration_velocity = 0
        accelerate = True
        deccelerate = False
        while distance > error:
            if accelerate:  # ACCELERATION PHASE
                # Accelerate to reach max velocity
                velocity += acceleration * rate.sleep_dur.to_sec()
                if velocity > max_velocity:
                    velocity = max_velocity
                    accelerate = False
                # Measure the distance needed to fully accelerate
                acceleration_distance += velocity * rate.sleep_dur.to_sec()
                if distance < acceleration_distance:
                    # Start deccelerating if the target point is closer than the distance needed for full stop
                    deccelerate = True
                    accelerate = False
                    decceleration_distance = distance
                    decceleration_velocity = velocity
            if not accelerate and not deccelerate:  # CONSTANT VELOCITY PHASE
                # Start deccelerating if the target point is closer than the distance needed for full stop
                if distance < acceleration_distance:
                    deccelerate = True
                    decceleration_distance = distance
                    decceleration_velocity = velocity
            if deccelerate:  # DECCElERATION PHASE
                # deccelerate until stopped
                velocity = decceleration_velocity * ((distance / decceleration_distance))**0.5

            # Pose update
            pose.x += direction[0] * velocity * rate.sleep_dur.to_sec()
            pose.y += direction[1] * velocity * rate.sleep_dur.to_sec()
            pose.z += direction[2] * velocity * rate.sleep_dur.to_sec()
            pose.pitch += direction[3] * velocity * rate.sleep_dur.to_sec()

            # Move to new pose
            self._move(pose)

            # Calculate distance to target
            distance = math.sqrt((pose.x - target_pose.x)**2 + (pose.y - target_pose.y)**2 +
                                 (pose.z - target_pose.z)**2 + (pose.pitch - target_pose.pitch)**2)
            rate.sleep()

        err = math.sqrt((pose.x - target_pose.x)**2 + (pose.y - target_pose.y)**2 + (pose.z - target_pose.z)**2)
        rospy.loginfo("Reached target point. Position error: %f m" % err)

    # For executing long movements. No chance of big trouble
    def move_jointspace(self, target_pose: ManipPose):
        jointstate = self.solver.get_IK_solution(self.get_pose())
        target_jointstate = self.solver.get_IK_solution(target_pose)

        # Calculate the vector of the movement
        direction = [
            target_jointstate.position[0] - jointstate.position[0],
            target_jointstate.position[1] - jointstate.position[1],
            target_jointstate.position[2] - jointstate.position[2],
            target_jointstate.position[3] - jointstate.position[3]
        ]
        distance = math.sqrt(direction[0]**2 + direction[1]**2 + direction[2]**2 + direction[3]**2)
        if distance == 0:
            print("Target point is the same as current point")
            return
        direction = [x / distance for x in direction]

        # Load movement parameters
        acceleration = self.MODES_DATA["jointspace"]["acceleration"]
        max_velocity = self.MODES_DATA["jointspace"]["max_velocity"]
        error = self.MODES_DATA["jointspace"]["max_error"]
        rate = rospy.Rate(self.MODES_DATA["jointspace"]["interpolation_rate"])

        velocity = 0.0
        acceleration_distance = 0
        decceleration_distance = 0
        decceleration_velocity = 0
        accelerate = True
        deccelerate = False
        while distance > error:
            if accelerate:  # ACCELERATION PHASE
                # Accelerate to reach max velocity
                velocity += acceleration * rate.sleep_dur.to_sec()
                if velocity > max_velocity:
                    velocity = max_velocity
                    accelerate = False
                # Measure the distance needed to fully accelerate
                acceleration_distance += velocity * rate.sleep_dur.to_sec()
                if distance < acceleration_distance:
                    # Start deccelerating if the target point is closer than the distance needed for full stop
                    deccelerate = True
                    accelerate = False
                    decceleration_distance = distance
                    decceleration_velocity = velocity
            if not accelerate and not deccelerate:  # CONSTANT VELOCITY PHASE
                # Start deccelerating if the target point is closer than the distance needed for full stop
                if distance < acceleration_distance:
                    deccelerate = True
                    decceleration_distance = distance
                    decceleration_velocity = velocity
            if deccelerate:  # DECCElERATION PHASE
                # deccelerate until stopped
                velocity = decceleration_velocity * ((distance / decceleration_distance))**0.5

            # jointstate update
            jointstate.position[0] += direction[0] * velocity * rate.sleep_dur.to_sec()
            jointstate.position[1] += direction[1] * velocity * rate.sleep_dur.to_sec()
            jointstate.position[2] += direction[2] * velocity * rate.sleep_dur.to_sec()
            jointstate.position[3] += direction[3] * velocity * rate.sleep_dur.to_sec()

            # Move to new jointstate
            self._publish_jointstate(jointstate)
            # TODO read actual joint state, remove the line below when it's done

            # Calculate distance to target
            distance = math.sqrt((jointstate.position[0] - target_jointstate.position[0])**2 +
                                 (jointstate.position[1] - target_jointstate.position[1])**2 +
                                 (jointstate.position[2] - target_jointstate.position[2])**2 +
                                 (jointstate.position[3] - target_jointstate.position[3])**2)
            rate.sleep()

        rospy.loginfo("Reached target point.")
        self.currentPos = target_pose

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("manip_controller")
    interface = ROSManipInterface()
    SiriusManip(interface).run()
