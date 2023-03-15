import pytest

import ik

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