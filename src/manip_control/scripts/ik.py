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
        self.flex_pitch = False

    def set_mode(self, mode: str) -> bool:
        if mode == "elbow_up":
            self.elbow_up = True
        elif mode == "elbow_down":
            self.elbow_up = False
        elif mode == "tilt_forward":
            self.tilt_forward = True
        elif mode == "tilt_backward":
            self.tilt_forward = False
        elif mode == "flex":
            self.flex_pitch = True
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
        l = self.lengths
        x_t = target.x
        y_t = target.y

        # The reasoning behind this code is explained in docs/Inverse_Kinematics_Formula.pdf

        # TODO: Resolve problem with step change of angle (now: 179 deg + 2 deg = -179 deg)
        if self.tilt_forward:
            # Last joint value simply equals to its target value
            solution[4] = target.roll
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
            solution[4] = math.pi - target.roll

        # Check if the calculated angles are within manipulator's limits
        if not self._angles_within_constraints(solution, [0, 4]):
            raise Exception("IK solution outside of joint limits!")

        # Translate, so link 0 length does not need to be considered in formulas below
        z_t = target.z - self.lengths[0]
        # Find 3rd link's start position given its absolute angle (alpha) and end position (r_t, z_t)
        r_3, z_3 = self._get_link3_startposition(r_t, z_t, alpha)
        outter_range = (l[1] + l[2])**2 <= r_3**2 + z_3**2
        inner_range = (l[1] - l[2])**2 >= r_3**2 + z_3**2

        if not outter_range and not inner_range:
            # Find 2nd link's start position
            # The geometry constraints lead to equation system with two solutions
            # One of them is chosen by 'elbow_up' argumet
            r_2, z_2 = self._find_middlepoint_solution(l[1], l[2], r_3, z_3, self.elbow_up)
            # Given the position, angles of the joints can be calculated
            solution[1], solution[2], solution[3] = self._calculate_angles(r_3, z_3, r_2, z_2, alpha)

            if self._angles_within_constraints(solution, [1, 2, 3]):
                return ManipJointState.from_list(solution)
            elif not self.flex_pitch:
                raise Exception("IK solution outside of joints limits!")

        elif not self.flex_pitch:
            raise Exception("IK solution outside of manip range!")

        solution[1], solution[2], solution[3] = self._get_nearest_solution(r_t, z_t, alpha)

        return ManipJointState.from_list(solution)

    def _get_nearest_solution(self, r_t: float, z_t: float, alpha: float):
        d_2 = r_t**2 + z_t**2
        limbs = self.lengths[1:4].copy()
        limbs.sort()
        dead_zone = limbs[-1] - sum(limbs[:-1])

        if sum(limbs)**2 <= d_2 or (dead_zone > 0.0 and d_2 <= dead_zone**2):
            raise Exception("IK solution outside of manip range!")

        critical_solutions = [
            self._find_solution_with_set_angle(index, angle, r_t, z_t, elbow_up)
            for index, (lower, upper) in enumerate(self.limits) if index >= 1 and index <= 3
            for angle in [lower, upper] + ([0.0, math.pi] if index != 1 else []) for elbow_up in [True, False]
        ]
        proper_solutions = [
            s for s in critical_solutions if s is not None and self._angles_within_constraints([0.0] + s, [1, 2, 3])
        ]

        if len(proper_solutions) == 0:
            raise Exception("IK solution outside of joints limits!")

        error = [abs(self._normalize_angle_range(sum(s) - math.pi / 2 - alpha)) for s in proper_solutions]
        best_solution = proper_solutions[error.index(min(error))]

        return best_solution

    def _find_solution_with_set_angle(self, index: int, angle: float, r_t: float, z_t: float, elbow_up: bool):
        l = self.lengths
        try:
            if index == 1:
                angle_1 = angle
                r_2 = math.sin(angle_1)
                z_2 = math.cos(angle_1)
                dr_4, dz_4 = r_t - r_2, z_t - z_2
                dr_3, dz_3 = self._find_middlepoint_solution(l[2], l[3], dr_4, dz_4, elbow_up)
                angle_2 = math.atan2(dr_3, dz_3) - angle_1
                angle_3 = math.atan2(dr_4 - dr_3, dz_4 - dz_3) - angle_1 - angle_2

            elif index == 2:
                angle_2 = angle
                l_t = math.sqrt(l[1]**2 + l[2]**2 - 2 * l[1] * l[2] * math.cos(math.pi + angle_2))
                beta = math.acos((l[1]**2 + l_t**2 - l[2]**2) / (2 * l_t * l[1]))
                r_3, z_3 = self._find_middlepoint_solution(l_t, l[3], r_t, z_t, elbow_up)
                angle_1 = math.atan2(r_3, z_3) - beta
                angle_3 = math.atan2(r_t - r_3, z_t - z_3) - angle_1 - angle_2

            elif index == 3:
                angle_3 = angle
                l_t = math.sqrt(l[2]**2 + l[3]**2 - 2 * l[2] * l[3] * math.cos(math.pi + angle_3))
                beta = math.acos((l[2]**2 + l_t**2 - l[3]**2) / (2 * l_t * l[2]))
                r_2, z_2 = self._find_middlepoint_solution(l[1], l_t, r_t, z_t, elbow_up)
                angle_1 = math.atan2(r_2, z_2)
                angle_2 = math.atan2(r_t - r_2, z_t - z_2) - angle_1 + beta

            angles = [self._normalize_angle_range(s) for s in [angle_1, angle_2, angle_3]]
            return angles

        except:
            return None

    def _angles_within_constraints(self, solution: 'list[float]', indexes: 'list[int]'):
        return all([checkBounds(solution[i], self.limits[i]) for i in indexes])

    def _get_link3_startposition(self, r_t: float, z_t: float, alpha: float):
        r_3 = r_t - self.lengths[3] * math.cos(alpha)
        z_3 = z_t + self.lengths[3] * math.sin(alpha)
        return r_3, z_3

    def _find_middlepoint_solution(self, a: float, b: float, x: float, y: float, middle_joint_up: bool):
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

    def _normalize_angle_range(self, angle: float):
        return math.pi * (((angle / math.pi % 2) + 3) % 2 - 1)

    def _initialize_solution(self):
        solution = [0.0] * len(self.limits)
        return solution


def checkBounds(value: float, bounds: 'tuple[float, float]'):
    return value >= bounds[0] and value <= bounds[1]
