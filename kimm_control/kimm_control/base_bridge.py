from abc import ABC, abstractmethod


class BaseBridge(ABC):
    """Base class for simulated and real hardware bridges.

    This intentionally resembles a future ros2_control-style boundary,
    but stays simple and Python-only for now.
    """

    @abstractmethod
    def configure(self):
        pass

    @abstractmethod
    def activate(self):
        pass

    @abstractmethod
    def deactivate(self):
        pass

    @abstractmethod
    def read(self):
        pass

    @abstractmethod
    def write_pose(self, pose):
        pass

    @abstractmethod
    def close(self):
        pass