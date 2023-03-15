from manip_interface import ManipInterface
from motion_interpolation import InterpolationSettings, MotionInterpolator
from ik import IKSolver, ManipPose


class MotionStrategy:

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
        position = self._calculate_start_coords(manip_interface)
        end_position = self._calculate_end_coords(manip_interface)
        self.motion_interpolator.set_movement(position, end_position)

        while self.motion_interpolator.is_not_done():
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
        manip_interface.set_jointstate(jointstate)


class JointspaceMotion(InterpolatedMotion):

    def _calculate_start_coords(self, manip_interface: ManipInterface):
        jointstate = manip_interface.get_jointstate()
        return jointstate.position

    def _calculate_end_coords(self, manip_interface: ManipInterface):
        jointstate = self.ik_solver.get_IK_solution(self.target_pose)
        return jointstate.position

    def _move_to_position(self, position, manip_interface: ManipInterface):
        jointstate = manip_interface.get_jointstate()
        jointstate.position = position
        manip_interface.set_jointstate(jointstate)
