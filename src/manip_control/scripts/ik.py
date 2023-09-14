#!/usr/bin/python3
import math
from abc import ABC, abstractmethod


class ManipPose:

    def __init__(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def from_list(data: 'list[float]'):
        return ManipPose(*data)

    def to_list(self):
        return [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]


class ManipJointState:

    def __init__(self, joint_value_list: 'list[float]'):
        self.position = joint_value_list

    def from_list(data: 'list[float]'):
        return ManipJointState(data)

    def to_list(self):
        return self.position


class IKSolver(ABC):

    def __init__(self, names: 'list[str]', lengths: 'list[float]', limits: 'list[tuple[float, float]]'):
        self.joint_names = names
        self.limits = limits
        self.lengths = lengths

    @abstractmethod
    def get_IK_solution(self, target: ManipPose) -> ManipJointState:
        pass

    @abstractmethod
    def get_FK_solution(self, jointstate: ManipJointState) -> ManipPose:
        pass

    @abstractmethod
    def set_mode(self, mode) -> None:
        pass


# Class implementing kinematics of SiriusII manipulator
class SiriusII_IKSolver(IKSolver):

    def __init__(self, names: 'list[str]', lengths: 'list[float]', limits: 'list[tuple[float, float]]'):
        super().__init__(names, lengths, limits)
        # For now, due to physical constraints of the manipulator only one of four IK solutions is reasonable
        self.elbow_up = True
        self.tilt_forward = True

    def set_mode(self, mode: str) -> bool:
        if mode == "elbow_up":
            self.elbow_up = True
        elif mode == "elbow_down":
            self.elbow_up = False
        elif mode == "tilt_forward":
            self.tilt_forward = True
        elif mode == "tilt_backward":
            self.tilt_forward = False
        else:
            return False
        return True

    def get_FK_solution(self, jointstate: ManipJointState) -> ManipPose:
        l = self.lengths
        angles = jointstate.position
        fi1 = angles[1]
        fi2 = fi1 + angles[2]
        fi3 = fi2 + angles[3]
        solution = ManipPose()

        # Calculate solution using SiriusII-specific FK formula
        r = l[1] * math.sin(fi1) + l[2] * math.sin(fi2) + l[3] * math.sin(fi3)
        z = l[1] * math.cos(fi1) + l[2] * math.cos(fi2) + l[3] * math.cos(fi3)
        solution.z = z + l[0]
        solution.x = r * math.cos(angles[0])
        solution.y = r * math.sin(angles[0])
        solution.pitch = fi3 - 0.5 * math.pi
        solution.roll = angles[4]

        return solution

    def get_IK_solution(self, target: ManipPose) -> ManipJointState:
        try:
            return self._calculate_IK_solution(target)
        except ValueError:
            raise Exception("No IK solution! Possibly out of range")
        except Exception as e:
            raise e

    def _calculate_IK_solution(self, target: ManipPose) -> ManipJointState:

        solution = self._initialize_solution()
        x_t = target.x
        y_t = target.y

        # The reasoning behind this code is explained in docs/Inverse_Kinematics_Formula.pdf

        # Last joint value simply equals to its target value
        solution[4] = target.roll

        # TODO: Resolve problem with step change of angle (now: 179 deg + 2 deg = -179 deg)
        if self.tilt_forward:
            # First joint angle can be easily obtained as it is the only joint allowing for movement along y-axis
            solution[0] = math.atan2(y_t, x_t)
            # Project target position onto manip plane reducing the issue to finding the pose of 3 planar links
            r_t = math.sqrt(x_t * x_t + y_t * y_t)
            alpha = target.pitch
        else:
            # Reverse manip and tilt back
            solution[0] = math.atan2(-y_t, -x_t)
            r_t = -math.sqrt(x_t * x_t + y_t * y_t)
            alpha = math.pi - target.pitch

        # Translate, so link 0 length does not need to be considered in formulas below
        z_t = target.z - self.lengths[0]

        solution[1], solution[2], solution[3] = self._get_3_links_solution(self.lengths, r_t, z_t, alpha, self.elbow_up)

        # Check if the calculated angles are within manipulator's limits
        if self._angles_within_constraints(solution):
            return ManipJointState.from_list(solution)

        raise Exception("IK solution outside of joints limits!")

    def _angles_within_constraints(self, solution: 'list[float]'):
        return all([checkBounds(s, b) for s, b in zip(solution, self.limits)])

    def _get_3_links_solution(self, lengths: 'list[float]', r_t: float, z_t: float, alpha: float, elbow_up: bool):
        # Find 3rd link's start position given its absolute angle (alpha) and end position (r_t, z_t)
        r_3, z_3 = self._get_link3_startposition(lengths, r_t, z_t, alpha)
        # Find 2nd link's start position
        # The geometry constraints lead to equation system with two solutions
        # One of them is chosen by 'elbow_up' argumet
        r_2, z_2 = self._find_middlepoint_solution(lengths[1], lengths[2], r_3, z_3, elbow_up)
        # Given the position, angles of the joints can be calculated
        solution = self._calculate_angles(r_3, z_3, r_2, z_2, alpha)

        return solution

    def _get_link3_startposition(self, r_t: float, z_t: float, alpha: float):
        r_3 = r_t - self.lengths[3] * math.cos(alpha)
        z_3 = z_t + self.lengths[3] * math.sin(alpha)
        return r_3, z_3

    def _find_middlepoint_solution(a: float, b: float, x: float, y: float, middle_joint_up: bool):
        # Standard names for elements of triangle with vertices A,C,B in subsequent joints
        c_2 = x * x + y * y
        u = ((a - b) * (a + b) + c_2) / 2  # u = a*c*cos(beta)
        ## v = a*b*sin(gamma) = a*c*sin(beta)
        v = math.sqrt((a * (b + a) - u) * (a * (b - a) + u))

        if middle_joint_up:
            r_2 = (x * u - y * v) / c_2
            z_2 = (x * v + y * u) / c_2
        else:
            r_2 = (x * u + y * v) / c_2
            z_2 = (-x * v + y * u) / c_2

        return r_2, z_2

    def _calculate_angles(self, r_3: float, z_3: float, r_2: float, z_2: float, alpha: float):
        angle1 = math.atan2(r_2, z_2)
        angle2 = math.atan2(r_3 - r_2, z_3 - z_2) - angle1
        angle3 = alpha + math.pi / 2 - angle1 - angle2
        angles = [angle1, angle2, angle3]
        return [self._normalize_angle_range(fi) for fi in angles]

    def _normalize_angle_range(angle: float):
        return math.pi * (((angle / math.pi % 2) + 3) % 2 - 1)

    def _initialize_solution(self):
        solution = [0.0] * len(self.limits)
        return solution


def checkBounds(value: float, bounds: 'tuple[float, float]'):
    return value >= bounds[0] and value <= bounds[1]
