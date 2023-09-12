from math import pi
from manip_interface import ManipInterface
from motion_interpolation import InterpolationSettings, MotionInterpolator
from ik import IKSolver, ManipPose

from abc import ABC, abstractmethod


class MotionStrategy(ABC):

    @abstractmethod
    def execute(self, manip_interface: ManipInterface):
        pass


class InterpolatedMotion(MotionStrategy):

    def __init__(self, target_pose: ManipPose, interpolation_settings: InterpolationSettings, ik_solver: IKSolver,
                 rate):
        self.motion_interpolator = MotionInterpolator(interpolation_settings)
        self.ik_solver = ik_solver
        self.target_pose = target_pose
        self.loop_delay = 1 / rate

    def execute(self, manip_interface: ManipInterface):
        self._initialize_execution(manip_interface)
        while self.motion_interpolator.is_not_done():
            self._step(manip_interface)

    def _initialize_execution(self, manip_interface: ManipInterface):
        position = self._calculate_start_coords(manip_interface)
        end_position = self._calculate_end_coords(manip_interface)
        self.motion_interpolator.set_movement(position, end_position)

    def _step(self, manip_interface: ManipInterface):
        position = self.motion_interpolator.movement_step(self.loop_delay)
        self._move_to_position(position, manip_interface)
        manip_interface.sleep(self.loop_delay)

    def _calculate_start_coords(self, manip_interface: ManipInterface):
        pass

    def _calculate_end_coords(self, manip_interface: ManipInterface):
        pass

    def _move_to_position(self, position, manip_interface: ManipInterface):
        pass

class CartesianMotion(InterpolatedMotion):

    def _calculate_start_coords(self, manip_interface: ManipInterface):
        pose = self.ik_solver.get_FK_solution(manip_interface.get_jointstate())
        return pose.to_list()

    def _calculate_end_coords(self, manip_interface: ManipInterface):
        pose = self.target_pose
        return pose.to_list()

    def _move_to_position(self, position, manip_interface: ManipInterface):
        pose = ManipPose.from_list(position)
        jointstate = self.ik_solver.get_IK_solution(pose)
        if jointstate=="-1":
            raise Exception("IK solution outside of joints limits")
        manip_interface.set_jointstate(jointstate)


class JointspaceMotion(InterpolatedMotion):

    def _calculate_start_coords(self, manip_interface: ManipInterface):
        jointstate = manip_interface.get_jointstate()
        return jointstate.position

    def _calculate_end_coords(self, manip_interface: ManipInterface):
        jointstate = self.ik_solver.get_IK_solution(self.target_pose)
        if jointstate=="-1":
            raise Exception("IK solution outside of joints limits")
        return jointstate.position

    def _move_to_position(self, position, manip_interface: ManipInterface):
        jointstate = manip_interface.get_jointstate()
        jointstate.position = position
        manip_interface.set_jointstate(jointstate)

class CartesianMotionExtRange(InterpolatedMotion):

    def _calculate_start_coords(self, manip_interface: ManipInterface):
        pose = self.ik_solver.get_FK_solution(manip_interface.get_jointstate())
        return pose.to_list()

    def _calculate_end_coords(self, manip_interface: ManipInterface):
        jointstate = self.ik_solver.get_IK_solution(self.target_pose)

        if jointstate=="-1":
            pitch_angle=pi/180
            while jointstate=="-1":
                self.target_pose.pitch=pitch_angle
                jointstate = self.ik_solver.get_IK_solution(self.target_pose)
                pitch_angle=pitch_angle+pi/180
                if pitch_angle>pi/2:
                    raise Exception("IK solution outside of joints limits")

        pose = self.target_pose
        return pose.to_list()

    def _move_to_position(self, position, manip_interface: ManipInterface):
        pose = ManipPose.from_list(position)
        jointstate = self.ik_solver.get_IK_solution(pose)
        if jointstate=="-1":
            raise Exception("IK solution outside of joints limits")
        manip_interface.set_jointstate(jointstate)


class JointspaceMotionExtRange(InterpolatedMotion):

    def _calculate_start_coords(self, manip_interface: ManipInterface):
        jointstate = manip_interface.get_jointstate()
        return jointstate.position

    def _calculate_end_coords(self, manip_interface: ManipInterface):
        jointstate = self.ik_solver.get_IK_solution(self.target_pose)

        if jointstate=="-1":
            pitch_angle=pi/180
            while jointstate=="-1":
                self.target_pose.pitch=pitch_angle
                jointstate = self.ik_solver.get_IK_solution(self.target_pose)
                pitch_angle=pitch_angle+pi/180
                if pitch_angle>pi/2:
                    raise Exception("IK solution outside of joints limits")


        return jointstate.position

    def _move_to_position(self, position, manip_interface: ManipInterface):
        jointstate = manip_interface.get_jointstate()
        jointstate.position = position
        manip_interface.set_jointstate(jointstate)

class IncrementalMotion(MotionStrategy):

    def __init__(self, delta: ManipPose, ik_solver: IKSolver):
        self.deltaPose = delta
        self.solver = ik_solver

    def execute(self, manip_interface: ManipInterface):
        currentJointstate = manip_interface.get_jointstate()
        currentPose = self.solver.get_FK_solution(currentJointstate)
        targetPose = self._add_poses(currentPose, self.deltaPose)
        targetJointstate = self.solver.get_IK_solution(targetPose)
        if targetJointstate=="-1":
            raise Exception("IK solution outside of joints limits")
        manip_interface.set_jointstate(targetJointstate)

    def _add_poses(self, pose1, pose2):
        return ManipPose.from_list([x1 + x2 for x1, x2 in zip(pose1.to_list(), pose2.to_list())])
    
class IncrementalMotionExtRange(MotionStrategy):
    
    def __init__(self, delta: ManipPose, ik_solver: IKSolver):
        self.deltaPose = delta
        self.solver = ik_solver
        

    def execute(self, manip_interface: ManipInterface):
        currentJointstate = manip_interface.get_jointstate()
        currentPose = self.solver.get_FK_solution(currentJointstate)
        targetPose = self._add_poses(currentPose, self.deltaPose)
        targetPose.pitch=0
        targetJointstate = self.solver.get_IK_solution(targetPose)
        
        if targetJointstate=="-1":
            pitch_angle=0
            while targetJointstate=="-1":
                pitch_angle = pitch_angle+pi/180
                targetPose.pitch = pitch_angle
                targetJointstate = self.solver.get_IK_solution(targetPose)
                if pitch_angle>pi/2:
                    raise Exception("IK solution outside of joints limits")
                
            auxAngle=pi/180/2
            count=0
            pitch_angle=pitch_angle-auxAngle
            lastSuccesfulSolution=targetJointstate

            while count <200:
                count=count+1
                targetPose.pitch=pitch_angle
                targetJointstate = self.solver.get_IK_solution(targetPose)
                auxAngle=auxAngle/2
                if targetJointstate=="-1":
                    pitch_angle=pitch_angle+auxAngle
                else:
                    pitch_angle=pitch_angle-auxAngle
                    lastSuccesfulSolution=targetJointstate
            
            targetJointstate=lastSuccesfulSolution
                
        manip_interface.set_jointstate(targetJointstate)

    def _add_poses(self, pose1, pose2):
            return ManipPose.from_list([x1 + x2 for x1, x2 in zip(pose1.to_list(), pose2.to_list())])