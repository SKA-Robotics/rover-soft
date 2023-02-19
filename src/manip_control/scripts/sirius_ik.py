#!/usr/bin/python3
import math
import graphics
import time
from sensor_msgs.msg import JointState


def checkBounds(value, bounds):
    return value >= bounds[0] and value <= bounds[1]


def list_to_jointstate(angles: list) -> JointState:
    j = JointState()
    j.position = [angle for angle in angles]
    return j


# Represents target point with rotation along one axis
class IKTarget:

    def __init__(self, x, y, z, alpha):
        self.x = x
        self.y = y
        self.z = z
        self.alpha = alpha


class IKSolver:

    def __init__(self, linkLengths: list, limits: list):
        self.limits = limits
        self.lengths = linkLengths

    def getIKSolution(self, target: IKTarget) -> JointState:
        solution = JointState()
        solution.position = [0] * 4
        limits = self.limits
        l = self.lengths

        x = target.x
        y = target.y
        solution.position[0] = math.atan2(y, x)
        if not checkBounds(solution.position[0], limits[0]):
            raise Exception("No IK solution!")

        d = math.sqrt(x * x + y * y)
        z = target.z - l[0]
        alpha = target.alpha

        dp = d - l[3] * math.cos(alpha)
        zp = z + l[3] * math.sin(alpha)

        dpp = (dp * dp + l[1] * l[1] - l[2] * l[2] + zp * zp -
               (zp *
                (dp * math.sqrt(
                    (l[1] * l[2] * 2.0 - dp * dp + l[1] * l[1] + l[2] * l[2] -
                     zp * zp) * (l[1] * l[2] * 2.0 + dp * dp - l[1] * l[1] -
                                 l[2] * l[2] + zp * zp)) + (dp * dp) * zp +
                 (l[1] * l[1]) * zp - (l[2] * l[2]) * zp + zp * zp * zp)) /
               (dp * dp + zp * zp)) / (dp * 2.0)
        zpp = (dp * math.sqrt(
            (l[1] * l[2] * 2.0 - dp * dp + l[1] * l[1] + l[2] * l[2] - zp * zp)
            * (l[1] * l[2] * 2.0 + dp * dp - l[1] * l[1] - l[2] * l[2] +
               zp * zp)) + (dp * dp) * zp + (l[1] * l[1]) * zp -
               (l[2] * l[2]) * zp + zp * zp * zp) / ((dp * dp) * 2.0 +
                                                     (zp * zp) * 2.0)

        beta = math.atan2(zpp, dpp)
        solution.position[1] = math.pi / 2 - beta
        gamma = math.atan2(zpp - zp, dp - dpp)
        solution.position[2] = beta + gamma
        solution.position[3] = alpha - gamma

        if all([
                checkBounds(solution.position[i], limits[i])
                for i in range(1, len(solution.position))
        ]):
            return solution

        dpp = (dp * dp + l[1] * l[1] - l[2] * l[2] + zp * zp -
               (zp *
                (-dp * math.sqrt(
                    (l[1] * l[2] * 2.0 - dp * dp + l[1] * l[1] + l[2] * l[2] -
                     zp * zp) * (l[1] * l[2] * 2.0 + dp * dp - l[1] * l[1] -
                                 l[2] * l[2] + zp * zp)) + (dp * dp) * zp +
                 (l[1] * l[1]) * zp - (l[2] * l[2]) * zp + zp * zp * zp)) /
               (dp * dp + zp * zp)) / (dp * 2.0)
        zpp = (-dp * math.sqrt(
            (l[1] * l[2] * 2.0 - dp * dp + l[1] * l[1] + l[2] * l[2] - zp * zp)
            * (l[1] * l[2] * 2.0 + dp * dp - l[1] * l[1] - l[2] * l[2] +
               zp * zp)) + (dp * dp) * zp + (l[1] * l[1]) * zp -
               (l[2] * l[2]) * zp + zp * zp * zp) / ((dp * dp) * 2.0 +
                                                     (zp * zp) * 2.0)
        beta = math.atan2(zpp, dpp)
        solution.position[1] = beta - math.pi / 2
        gamma = math.atan2(zpp - zp, dp - dpp)
        solution.position[2] = beta + gamma
        solution.position[3] = alpha - gamma

        if all([
                checkBounds(solution.position[i], limits[i])
                for i in range(1, len(solution.position))
        ]):
            return solution

        raise Exception("No IK solution!")


# Code below was used for debugging during implementation of analytical IK solution. Won't be needed
class Visualization:

    def __init__(self) -> None:
        self.window = graphics.GraphWin("IK solver", 500, 500)
        self.window.bind("<Motion>", self.motion)
        self.window.setBackground("white")
        self.mouse_x = 0
        self.mouse_y = 0

        self.links = [Link(1, None)]
        self.links.append(Link(3, self.links[-1]))
        self.links.append(Link(3, self.links[-1]))
        self.links.append(Link(1, self.links[-1]))
        self.links[0].localAngle = -math.pi / 2
        self.links[1].localAngle = math.pi / 6
        self.links[2].localAngle = math.pi / 2
        self.links[3].localAngle = -math.pi / 4

        self.target_angle = 0

    def motion(self, event):
        self.mouse_x = (event.x - 250) / 20
        self.mouse_y = -(event.y - 250) / 20

    def run(self):
        solver = IKSolver([1, 3, 3, 1], [(-3, 3), (-3, 3), (-3, 3), (-3, 3)])
        while True:

            key = self.window.checkKey()
            if key == "a":
                self.target_angle += 0.1
            if key == "d":
                self.target_angle -= 0.1

            for link in self.links:
                pos = link.get_end_position()
                self.window.plot(250 + pos[0] * 20, 250 - pos[1] * 20, "white")
            try:
                solution = solver.getIKSolution(
                    IKTarget(self.mouse_x, 0, self.mouse_y, self.target_angle)
                )
                for i in range(1, 4):
                    self.links[i].localAngle = solution.position[i]
                print(solution)
            except Exception as e:
                print(e)
                print(e.__traceback__.tb_next.tb_lineno)

            self.window.plot(250, 250)
            for link in self.links:
                pos = link.get_end_position()
                self.window.plot(250 + pos[0] * 20, 250 - pos[1] * 20)
            time.sleep(0.1)


class Link:

    def __init__(self, length, parent) -> None:
        self.length = length
        self.localAngle = 0
        self.parent = parent

    def get_global_angle(self):
        if self.parent is None:
            return self.localAngle
        else:
            return self.localAngle + self.parent.get_global_angle()

    def get_end_position(self):
        angle = self.get_global_angle()
        dx = self.length * math.cos(-angle)
        dy = self.length * math.sin(-angle)
        if self.parent is None:
            return (dx, dy)
        else:
            parentEndPos = self.parent.get_end_position()
            return (parentEndPos[0] + dx, parentEndPos[1] + dy)


if __name__ == "__main__":
    v = Visualization()
    v.run()
