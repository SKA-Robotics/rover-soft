#!/usr/bin/python3
import math


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


#TODO: Create abstract base class of kinematics solver


# Class implementing kinematics of SiriusII manipulator
class IKSolver:

    def __init__(self, names, lengths, limits):
        self.joint_names = names
        self.limits = limits
        self.lengths = lengths

    # Calculates angles of all joints given target position
    def get_IK_solution(self, target: ManipPose) -> ManipJointState:
        # Initialize solution JointState object
        solution = [0] * len(self.joint_names)

        limits = self.limits
        l = self.lengths
        x = target.x
        y = target.y

        solution[4] = target.roll
        # Calculate solution using SiriusII-specific IK formula
        # First joint angle can be easily obtained as it is
        # the only joint allowing for movement along y-axis
        solution[0] = math.atan2(y, x)
        if not checkBounds(solution.position[0], limits[0]):
            raise Exception("No IK solution!")

        # Project target position to manip plane
        d = math.sqrt(x * x + y * y)
        # Translate, so link 0 length does not need to be considered
        # in formulas below
        z = target.z - l[0]

        # Find 3rd link's start position given its absolute angle
        # (target.alpha) and end position (target position)
        alpha = target.pitch
        dp = d - l[3] * math.cos(alpha)
        zp = z + l[3] * math.sin(alpha)

        # Find 2nd link's start position
        # The geometry constraints lead to equation system with
        # two solutions. Due to physical constraints of the manipulator
        # only one of these solutions will be possible to reach
        dpp = (dp * dp + l[1] * l[1] - l[2] * l[2] + zp * zp - (zp * (dp * math.sqrt(
            (l[1] * l[2] * 2.0 - dp * dp + l[1] * l[1] + l[2] * l[2] - zp * zp) *
            (l[1] * l[2] * 2.0 + dp * dp - l[1] * l[1] - l[2] * l[2] + zp * zp)) + (dp * dp) * zp + (l[1] * l[1]) * zp -
                                                                      (l[2] * l[2]) * zp + zp * zp * zp)) /
               (dp * dp + zp * zp)) / (dp * 2.0)
        zpp = (dp * math.sqrt((l[1] * l[2] * 2.0 - dp * dp + l[1] * l[1] + l[2] * l[2] - zp * zp) *
                              (l[1] * l[2] * 2.0 + dp * dp - l[1] * l[1] - l[2] * l[2] + zp * zp)) + (dp * dp) * zp +
               (l[1] * l[1]) * zp - (l[2] * l[2]) * zp + zp * zp * zp) / ((dp * dp) * 2.0 + (zp * zp) * 2.0)

        # Given the position, angles of the joints can be calculated
        beta = math.atan2(zpp, dpp)
        solution[1] = math.pi / 2 - beta
        gamma = math.atan2(zpp - zp, dp - dpp)
        solution[2] = beta + gamma
        solution[3] = alpha - gamma

        # Check if the calculated angles are within manipulator's limits
        if all([checkBounds(solution[i], limits[i]) for i in range(1, len(solution))]):
            return ManipJointState.from_list(solution)

        # If the previous solution did not pass the check, try the second solution
        dpp = (dp * dp + l[1] * l[1] - l[2] * l[2] + zp * zp - (zp * (-dp * math.sqrt(
            (l[1] * l[2] * 2.0 - dp * dp + l[1] * l[1] + l[2] * l[2] - zp * zp) *
            (l[1] * l[2] * 2.0 + dp * dp - l[1] * l[1] - l[2] * l[2] + zp * zp)) + (dp * dp) * zp + (l[1] * l[1]) * zp -
                                                                      (l[2] * l[2]) * zp + zp * zp * zp)) /
               (dp * dp + zp * zp)) / (dp * 2.0)
        zpp = (-dp * math.sqrt((l[1] * l[2] * 2.0 - dp * dp + l[1] * l[1] + l[2] * l[2] - zp * zp) *
                               (l[1] * l[2] * 2.0 + dp * dp - l[1] * l[1] - l[2] * l[2] + zp * zp)) + (dp * dp) * zp +
               (l[1] * l[1]) * zp - (l[2] * l[2]) * zp + zp * zp * zp) / ((dp * dp) * 2.0 + (zp * zp) * 2.0)
        beta = math.atan2(zpp, dpp)
        solution[1] = beta - math.pi / 2
        gamma = math.atan2(zpp - zp, dp - dpp)
        solution[2] = beta + gamma
        solution[3] = alpha - gamma
        if all([checkBounds(solution[i], limits[i]) for i in range(1, len(solution))]):
            return ManipJointState.from_list(solution)

        # Both of the solutions are wrong. Exception is to be thrown
        raise Exception("No IK solution!")

    # Calculates manipulator's tip pose given joint angles
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