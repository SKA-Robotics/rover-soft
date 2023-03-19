from manip_interface import ManipInterface, ManipParams
from ik import ManipJointState

class DummyManipInterface(ManipInterface):

    def __init__(self):
        self.jointstate = None
        self._params = None

    def get_jointstate(self) -> ManipJointState:
        return self.jointstate

    def set_jointstate(self, jointstate: ManipJointState):
        self.jointstate = jointstate

    def sleep(self, time):
        pass

    def get_manip_params(self) -> ManipParams:
        return self._params

    def set_manip_params(self, params: ManipParams):
        self._params = params