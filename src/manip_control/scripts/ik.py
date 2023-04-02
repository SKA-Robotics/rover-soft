#!/usr/bin/python3
import math
from abc import ABC, abstractmethod


class ManipPose:

    def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def from_list(data):
        return ManipPose(*data)

    def to_list(self):
        return [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]


class ManipJointState:

    def __init__(self, joint_value_list):
        self.position = joint_value_list

    def from_list(data):
        return ManipJointState(data)

    def to_list(self):
        return self.position


class IKSolver(ABC):

    def __init__(self, names, lengths, limits):
        self.joint_names = names
        self.limits = limits
        self.lengths = lengths

    @abstractmethod
    def get_IK_solution(self, target: ManipPose) -> ManipJointState:
        pass

    @abstractmethod
    def get_FK_solution(self, jointstate: ManipJointState) -> ManipPose:
        pass


# Class implementing kinematics of SiriusII manipulator
class SiriusII_IKSolver(IKSolver):

    def get_IK_solution(self, target: ManipPose) -> ManipJointState:
        try:
            return self._calculate_IK_solution(target)
        except ValueError:
            raise Exception("No IK solution! Possibly out of range")
        except Exception as e:
            raise e

    def _calculate_IK_solution(self, target: ManipPose) -> ManipJointState:

        solution = self._initialize_solution()
        limits = self.limits
        l = self.lengths
        x = target.x
        y = target.y

        # Last joint value simply equals to its target value
        solution[4] = target.roll
        # First joint angle can be easily obtained as it is
        # the only joint allowing for movement along y-axis
        solution[0] = math.atan2(y, x)
        if not checkBounds(solution[0], limits[0]):
            raise Exception("IK solution outside of joint limits!")

        # Project target position to manip plane
        d = math.sqrt(x * x + y * y)
        # Translate, so link 0 length does not need to be considered
        # in formulas below
        z = target.z - l[0]

        # Find 3rd link's start position given its absolute angle
        # (target.alpha) and end position (target position)
        alpha = target.pitch
        dp, zp = self._get_link3_startposition(l, d, z, alpha)

        # Find 2nd link's start position
        # The geometry constraints lead to equation system with
        # two solutions. Due to physical constraints of the manipulator
        # only one of these solutions will be possible to reach
        dpp, zpp = self._find_middlepoint_solution1(l, dp, zp)
        # Given the position, angles of the joints can be calculated
        solution[1], solution[2], solution[3] = self._calculate_angles(target, dp, zp, dpp, zpp)
        # Check if the calculated angles are within manipulator's limits
        if self._angles_within_constraints(solution):
            return ManipJointState.from_list(solution)

        raise Exception("IK solution outside of joints limits")

    def _get_link3_startposition(self, l, d, z, alpha):
        dp = d - l[3] * math.cos(alpha)
        zp = z + l[3] * math.sin(alpha)
        return dp, zp

    def _angles_within_constraints(self, solution):
        return all([checkBounds(solution[i], self.limits[i]) for i in range(1, len(solution))])

    def _calculate_angles(self, target, dp, zp, dpp, zpp):
        alpha = target.pitch
        beta = math.atan2(zpp, dpp)
        gamma = math.atan2(zpp - zp, dp - dpp)
        angle1 = math.pi / 2 - beta
        angle2 = beta + gamma
        angle3 = alpha - gamma
        return angle1, angle2, angle3

    def _find_middlepoint_solution1(self, l, dp, zp):
        dpp = (dp * dp + l[1] * l[1] - l[2] * l[2] + zp * zp - (zp * (dp * math.sqrt(
            (l[1] * l[2] * 2.0 - dp * dp + l[1] * l[1] + l[2] * l[2] - zp * zp) *
            (l[1] * l[2] * 2.0 + dp * dp - l[1] * l[1] - l[2] * l[2] + zp * zp)) + (dp * dp) * zp + (l[1] * l[1]) * zp -
                                                                      (l[2] * l[2]) * zp + zp * zp * zp)) /
               (dp * dp + zp * zp)) / (dp * 2.0)
        zpp = (dp * math.sqrt((l[1] * l[2] * 2.0 - dp * dp + l[1] * l[1] + l[2] * l[2] - zp * zp) *
                              (l[1] * l[2] * 2.0 + dp * dp - l[1] * l[1] - l[2] * l[2] + zp * zp)) + (dp * dp) * zp +
               (l[1] * l[1]) * zp - (l[2] * l[2]) * zp + zp * zp * zp) / ((dp * dp) * 2.0 + (zp * zp) * 2.0)

        return dpp, zpp

    def _initialize_solution(self):
        solution = [0] * len(self.limits)
        return solution

    def get_FK_solution(self, jointstate: ManipJointState) -> ManipPose:
        angles = jointstate.position

        l = self.lengths
        solution = ManipPose()

        # Calculate solution using SiriusII-specific FK formula
        d = l[1] * math.sin(
            angles[1]) + l[2] * math.sin(angles[1] + angles[2]) + l[3] * math.sin(angles[1] + angles[2] + angles[3])
        z = l[1] * math.cos(
            angles[1]) + l[2] * math.cos(angles[1] + angles[2]) + l[3] * math.cos(angles[1] + angles[2] + angles[3])
        solution.z = z + l[0]
        solution.x = d * math.cos(angles[0])
        solution.y = d * math.sin(angles[0])
        solution.pitch = angles[1] + angles[2] + angles[3] - 0.5 * math.pi
        solution.roll = angles[4]

        return solution


def checkBounds(value, bounds):
    return value >= bounds[0] and value <= bounds[1]
