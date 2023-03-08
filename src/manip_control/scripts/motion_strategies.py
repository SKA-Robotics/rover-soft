import rospy
from sirius_msgs.msg import ManipPose

from ros_interface import ManipInterface
from motion_interpolation import InterpolationSettings, MotionInterpolator
from ik import IKSolver

class MotionStrategy:
    def execute(self, manip_interface : ManipInterface):
        pass


class InterpolatedMotion(MotionStrategy):
    def __init__(self, target_pose : ManipPose, interpolation_settings : InterpolationSettings, ik_solver : IKSolver, rate):
        self.motion_interpolator = MotionInterpolator(interpolation_settings)
        self.ik_solver = ik_solver
        self.target_pose = target_pose
        self.rate = rospy.Rate(rate)
        
    def execute(self, manip_interface: ManipInterface):
        position = self._calculate_start_coords(manip_interface)
        end_position = self._calculate_end_coords(manip_interface)
        self.motion_interpolator.set_movement(position, end_position)

        while self.motion_interpolator.is_not_done():
            position = self.motion_interpolator.movement_step(self.rate.sleep_dur.to_sec())
            self._move_to_position(position, manip_interface)
            self.rate.sleep()
        
    def _calculate_start_coords(self, manip_interface : ManipInterface):
        pass

    def _calculate_end_coords(self, manip_interface : ManipInterface):
        pass 

    def _move_to_position(self, position, manip_interface : ManipInterface):
        pass


class CartesianMotion(InterpolatedMotion):
    def _calculate_start_coords(self, manip_interface : ManipInterface):
        pose = self.ik_solver.get_FK_solution(manip_interface.get_jointstate())
        return [pose.x, pose.y, pose.z, pose.pitch]
    
    def _calculate_end_coords(self, manip_interface: ManipInterface):
        pose = self.target_pose
        return [pose.x, pose.y, pose.z, pose.pitch]

    def _move_to_position(self, position, manip_interface: ManipInterface):
        pose = ManipPose(position[0], position[1], position[2], position[3])
        jointstate = self.ik_solver.get_IK_solution(pose)
        manip_interface.set_jointstate(jointstate) 


class JointspaceMotion(InterpolatedMotion):
    def _calculate_start_coords(self, manip_interface: ManipInterface):
       full_jointstate = manip_interface.get_jointstate()
        # but the one returned from ROSManipInterface has all present in model.
        # This object cannot differentiate between needed and unneeded values, 
        # so instead ik_solver is abused. TODO: Make that cleaner.
       pose = self.ik_solver.get_FK_solution(full_jointstate)
       jointstate = self.ik_solver.get_IK_solution(pose)

       return jointstate.position
 
    def _calculate_end_coords(self, manip_interface: ManipInterface):
        jointstate = self.ik_solver.get_IK_solution(self.target_pose)
        return jointstate.position
    
    def _move_to_position(self, position, manip_interface: ManipInterface):
        full_jointstate = manip_interface.get_jointstate()
        # This is an ugly fix - expected jointstate should have only needed joints,
        # but the one returned from ROSManipInterface has all present in model.
        # This object cannot differentiate between needed and unneeded values, 
        # so instead ik_solver is abused. TODO: Make that cleaner.
        pose = self.ik_solver.get_FK_solution(full_jointstate)
        jointstate = self.ik_solver.get_IK_solution(pose)
        jointstate.position = position

        manip_interface.set_jointstate(jointstate)
        pose = self.ik_solver.get_FK_solution(jointstate)