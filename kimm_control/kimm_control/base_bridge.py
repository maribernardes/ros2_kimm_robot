from abc import ABC, abstractmethod


class BaseBridge(ABC):
    """Base class for KIMM simulated and hardware bridges.

    This is a small Python-only interface inspired by the ros2_control
    hardware lifecycle. It operates at actuator-joint level.

    In the current package, the bridge may be implemented directly as a ROS2
    node. This interface documents the lifecycle and method names that can be
    migrated later into a ros2_control hardware interface.
    """

    @abstractmethod
    def configure(self):
        """Prepare the simulated or hardware backend without enabling motion."""
        pass

    @abstractmethod
    def activate(self):
        """Enable command acceptance / hardware actuation."""
        pass

    @abstractmethod
    def deactivate(self):
        """Disable command acceptance / hardware actuation."""
        pass

    @abstractmethod
    def read(self):
        """Return actuator-level state."""
        pass

    @abstractmethod
    def write(self, joint_positions):
        """Command actuator joint positions."""
        pass

    @abstractmethod
    def close(self):
        """Cleanly release backend resources."""
        pass
