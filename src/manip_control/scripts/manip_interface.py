from ik import ManipJointState


class ManipInterface:

    def get_jointstate(self) -> ManipJointState:
        pass

    def set_jointstate(self, jointstate: ManipJointState):
        pass

    def get_manip_params(self):
        pass
