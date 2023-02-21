#!/usr/bin/python3
import rospy
import math
import random
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from sirius_msgs.msg import ManipPose
from std_msgs.msg import Float32

from sirius_ik import IKSolver


# Class for controlling the manipulator
class SiriusManip:

    def __init__(self):
        rospy.init_node("manip_controller")

        self.MODES_DATA = rospy.get_param("~control_modes")
        self.LINKS_DATA = rospy.get_param("~links")
        self.solver = IKSolver(
            self.LINKS_DATA["names"],
            self.LINKS_DATA["lengths"],
            self.LINKS_DATA["limits"]
            )

        rospy.Subscriber("/clicked_point",
                         PointStamped,
                         self.point_callback,
                         queue_size=10)
        self.point_publisher = rospy.Publisher("/received_point",
                                               PointStamped,
                                               queue_size=10)
        self.jointstate_publisher = rospy.Publisher("/joint_states",
                                                    JointState,
                                                    queue_size=10)
        self.rate = rospy.Rate(10.0)

        # TODO read actual joint state, remove the line below when it's done
        self.currentPos = ManipPose()

    def point_callback(self, data):
        rospy.loginfo("Received request:\n" + str(data.point))
        target = ManipPose()
        target.x = data.point.x
        target.y = data.point.y
        target.z = data.point.z
        target.pitch = random.uniform(-math.pi/4, 0)

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
            rospy.logwarn(e)
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
        jointstate.header.stamp = rospy.Time.now()
        self.jointstate_publisher.publish(jointstate)
    
    def get_pose(self) -> ManipPose:
        # TODO read actual joint state
        return self.currentPos
    
    def get_jointstate(self) -> JointState:
        # TODO read actual joint state
        return self.solver.get_IK_solution(self.currentPos)

    def move_cartesian(self, target_pose: ManipPose):
        pose = self.get_pose()

        # Calculate the vector of the movement
        direction = [
            target_pose.x - pose.x,
            target_pose.y - pose.y,
            target_pose.z - pose.z,
            target_pose.pitch - pose.pitch
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
        deceleration_distance = 0
        deceleration_velocity = 0
        accelerate = True
        decelerate = False
        while distance > error:
            if accelerate: # ACCELERATION PHASE
                # Accelerate to reach max velocity
                velocity += acceleration * rate.sleep_dur.to_sec()
                if velocity > max_velocity:
                    velocity = max_velocity
                    accelerate = False
                # Measure the distance needed to fully accelerate
                acceleration_distance += velocity * rate.sleep_dur.to_sec()
                if distance < acceleration_distance:
                    # Start decelerating if the target point is closer than the distance needed for full stop
                    decelerate = True
                    accelerate = False
                    deceleration_distance = distance
                    deceleration_velocity = velocity
            if not accelerate and not decelerate: # CONSTANT VELOCITY PHASE
                # Start decelerating if the target point is closer than the distance needed for full stop
                if distance < acceleration_distance:
                    decelerate = True
                    deceleration_distance = distance
                    deceleration_velocity = velocity
            if decelerate: # DECELERATION PHASE
                # Decelerate until stopped
                velocity = deceleration_velocity * ((distance / deceleration_distance)) ** 0.5

            # Pose update
            pose.x += direction[0] * velocity * rate.sleep_dur.to_sec()
            pose.y += direction[1] * velocity * rate.sleep_dur.to_sec()
            pose.z += direction[2] * velocity * rate.sleep_dur.to_sec()
            pose.pitch += direction[3] * velocity * rate.sleep_dur.to_sec()

            # Move to new pose
            self._move(pose)

            # Calculate distance to target
            distance = math.sqrt((pose.x - target_pose.x)**2 + (pose.y - target_pose.y)**2 + (pose.z - target_pose.z)**2 + (pose.pitch - target_pose.pitch)**2)
            rate.sleep()
        
        err = math.sqrt((pose.x - target_pose.x)**2 + (pose.y - target_pose.y)**2 + (pose.z - target_pose.z)**2)
        rospy.loginfo("Reached target point. Position error: %f m" % err)

    # TODO implement jointspace movement

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    SiriusManip().run()
