from ik import ManipJointState


class ManipInterface:

    def get_jointstate(self) -> ManipJointState:
        pass

    def set_jointstate(self, jointstate: ManipJointState):
        pass

    def sleep(self, time):
        pass

    def get_manip_params(self):
        pass


class DummyManipInterface:

    def __init__(self):
        self.jointstate = None

    def get_jointstate(self) -> ManipJointState:
        return self.jointstate

    def set_jointstate(self, jointstate: ManipJointState):
        self.jointstate = jointstate
