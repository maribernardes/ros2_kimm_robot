import ctypes
from pathlib import Path
import os


class KimmKinematics:
    """ctypes wrapper around the KIMM Stewart platform kinematics library.

    This class only handles kinematics.
    It does not communicate with hardware.
    """

    def __init__(self, library_path):
        self.library_path = Path(library_path)
        self._lib = None
        self._handle = None

    def configure(self):
        if not self.library_path.exists():
            raise FileNotFoundError(
                f"KIMM kinematics library not found: {self.library_path}"
            )

        self._lib = ctypes.CDLL(str(self.library_path))

        self._lib.spkernel_new.argtypes = []
        self._lib.spkernel_new.restype = ctypes.c_void_p

        self._lib.spkernel_delete.argtypes = [ctypes.c_void_p]
        self._lib.spkernel_delete.restype = None

        self._lib.initialize_robot_params.argtypes = [ctypes.c_void_p]
        self._lib.initialize_robot_params.restype = ctypes.c_int

        self._lib.reset_position.argtypes = [ctypes.c_void_p]
        self._lib.reset_position.restype = None

        self._lib.set_position_input_all.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_double),
        ]
        self._lib.set_position_input_all.restype = None

        self._lib.reverse_kinetics.argtypes = [ctypes.c_void_p]
        self._lib.reverse_kinetics.restype = ctypes.c_int

        self._lib.get_servo_angle_rad_joint_value.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int32,
        ]
        self._lib.get_servo_angle_rad_joint_value.restype = ctypes.c_double

        self._handle = self._lib.spkernel_new()
        if self._handle is None:
            raise RuntimeError("Failed to create KIMM kinematics instance.")

        old_cwd = os.getcwd()
        try:
            os.chdir(str(self.library_path.parent))
            result = self._lib.initialize_robot_params(self._handle)
        finally:
            os.chdir(old_cwd)

        if result not in (0, 1):
            raise RuntimeError(
                f"initialize_robot_params() returned unexpected code {result}. "
                f"Library path: {self.library_path}. "
                f"Initialization cwd: {self.library_path.parent}"
            )

    def close(self):
        if self._lib is not None and self._handle is not None:
            self._lib.spkernel_delete(self._handle)
            self._handle = None

    def solve_ik(self, kimm_input):
        """Run inverse kinematics.

        Args:
            kimm_input:
                [Rx_deg, Ry_deg, Rz_deg, Tx_mm, Ty_mm, Tz_mm]

        Returns:
            List of six actuator angles in radians.
        """

        if self._lib is None or self._handle is None:
            raise RuntimeError("KIMM kinematics library is not configured.")

        if len(kimm_input) != 6:
            raise ValueError("KIMM input must contain exactly 6 values.")

        input_array = (ctypes.c_double * 6)(*kimm_input)

        self._lib.set_position_input_all(self._handle, input_array)
        result = self._lib.reverse_kinetics(self._handle)

        if result not in (0, 1):
            raise RuntimeError(
                f"reverse_kinetics() returned unexpected code {result}"
            )

        return [
            float(self._lib.get_servo_angle_rad_joint_value(self._handle, i))
            for i in range(6)
        ]