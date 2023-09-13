from ik import ManipJointState

from abc import ABC, abstractmethod


class ManipParams:

    def __init__(self, joint_names: 'list[str]', joint_limits: 'list[float]', link_lengths: 'list[tuple[float, float]]',
                 control_modes: 'dict[str, float]') -> None:
        self._joint_names = joint_names
        self._joint_limits = joint_limits
        self._link_lengths = link_lengths
        self._control_modes = control_modes

    def from_dict(dict: 'dict[str, dict[str, float | list[str | float | tuple[float, float]]]]'):
        link_dict = dict["links"]
        joint_names = link_dict["names"]
        joint_limits = link_dict["limits"]
        link_lengths = link_dict["lengths"]
        control_modes = dict["control_modes"]
        result = ManipParams(joint_names, joint_limits, link_lengths, control_modes)
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
