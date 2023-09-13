from ik import ManipJointState

from abc import ABC, abstractmethod


class ManipParams:

    def from_dict(dict: 'dict[str, dict[str, float | list[str | float | tuple[float, float]]]]'):
        result = ManipParams()
        link_dict = dict["links"]
        result._joint_names = link_dict["names"]
        result._joint_limits = link_dict["limits"]
        result._link_lengths = link_dict["lengths"]
        result._control_modes = dict["control_modes"]
        return result

    def joint_names(self) -> 'list[str]':
        return self._joint_names

    def joint_limits(self) -> 'list[float]':
        return self._joint_limits

    def link_lengths(self) -> 'list[tuple[float, float]]':
        return self._link_lengths

    def control_mode_params(self, name: str) -> dict:
        return self._control_modes[name]


class ManipInterface(ABC):

    @abstractmethod
    def get_jointstate(self) -> ManipJointState:
        pass

    @abstractmethod
    def set_jointstate(self, jointstate: ManipJointState):
        pass

    @abstractmethod
    def sleep(self, time: float):
        pass

    @abstractmethod
    def get_manip_params(self) -> ManipParams:
        pass
