import pytest
import math

import ik
import manip_interface


def assert_equals(given, expected):
    assert given == pytest.approx(expected)


def assert_list_equals(given, expected):
    for g, e in zip(given, expected):
        assert_equals(g, e)


def test_alwaysOk():
    assert True


def test_manipPose_listConversionNoSideEffect():
    data = [1.2, 0.3, 0.2, 0.8, 0.2, 0.4]
    pose = ik.ManipPose.from_list(data)
    poseData = pose.to_list()
    assert_list_equals(data, poseData)


def test_manipJointState_listConversionNoSideEffect():
    data = [1.2, 0.3, 0.2, 0.4, 0.8]
    jointstate = ik.ManipJointState.from_list(data)
    jointstate_data = jointstate.to_list()
    assert_list_equals(data, jointstate_data)


class TestIKSolver:
    solver = ik.IKSolver([], [0.0655, 0.4350, 0.4650, 0.129], [(-3.0, 3.0), (-3.0, 3.0), (-3.0, 3.0), (-3.0, 3.0),
                                                               (-3.0, 3.0)])
    example_pose = ik.ManipPose.from_list([0.5, 0.1, 0.2, 0.8, 0.6, 0.0])
    example_pose_jointstate = ik.ManipJointState.from_list(
        [0.19739555984988078, -0.00465756979259524, 2.0870661094695873, 0.08838778711790474, 0.8])

    def test_IK_solution(self):
        jointstate = self.solver.get_IK_solution(self.example_pose)
        assert_list_equals(jointstate.to_list(), self.example_pose_jointstate.to_list())

    def test_FK_solution(self):
        pose = self.solver.get_FK_solution(self.example_pose_jointstate)
        assert_list_equals(pose.to_list(), self.example_pose.to_list())

    def test_backAndForth(self):
        jointstate = self.solver.get_IK_solution(self.example_pose)
        new_pose = self.solver.get_FK_solution(jointstate)
        assert_list_equals(new_pose.to_list(), self.example_pose.to_list())


class TestManipInterface:
    interface = manip_interface.DummyManipInterface()

    def test_setgetJointstate(self):
        jointstate = ik.ManipJointState([0.0, 0.1, 1.0, -0.1, 0.1])
        self.interface.set_jointstate(jointstate)
        received_jointstate = self.interface.get_jointstate()
        assert_list_equals(received_jointstate.to_list(), jointstate.to_list())
