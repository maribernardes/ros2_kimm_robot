from kimm_control.conversions import pose_dict_to_kimm_input
from kimm_control.base_bridge import BaseBridge


class SimulatedBridge(BaseBridge):
    """Simulated hardware bridge.

    This bridge still uses the real KIMM kinematics library.
    It only simulates hardware execution.
    """

    def __init__(self, kinematics, home_pose):
        self.kinematics = kinematics
        self.home_pose = dict(home_pose)
        self.enabled = False
        self.current_pose = dict(home_pose)
        self.actuator_positions_rad = [0.0] * 6

    def configure(self):
        self.kinematics.configure()

        # Initialize internal simulated state at home.
        kimm_input = pose_dict_to_kimm_input(self.home_pose)
        self.actuator_positions_rad = self.kinematics.solve_ik(kimm_input)
        self.current_pose = dict(self.home_pose)

    def activate(self):
        self.enabled = True

    def deactivate(self):
        self.enabled = False

    def read(self):
        return {
            "enabled": self.enabled,
            "pose": dict(self.current_pose),
            "actuator_positions_rad": list(self.actuator_positions_rad),
        }

    def write_pose(self, pose):
        kimm_input = pose_dict_to_kimm_input(pose)
        actuator_positions_rad = self.kinematics.solve_ik(kimm_input)

        self.current_pose = dict(pose)
        self.actuator_positions_rad = list(actuator_positions_rad)

        return self.read()

    def close(self):
        self.kinematics.close()