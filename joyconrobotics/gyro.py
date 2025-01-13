from .wrappers import PythonicJoyCon
from glm import vec2, vec3, quat, angleAxis, eulerAngles
from typing import Optional
import time


class GyroTrackingJoyCon(PythonicJoyCon):
    """
    A specialized class based on PythonicJoyCon which tracks the gyroscope data
    and deduces the current rotation of the JoyCon. Can be used to create a
    pointer rotate an object or pointin a direction. Comes with the need to be
    calibrated.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, simple_mode=False, **kwargs)

        # set internal state:
        self.reset_orientation()

        # register the update callback
        self.register_update_hook(self._gyro_update_hook)

    @property
    def pointer(self) -> Optional[vec2]:
        d = self.direction
        if d.x <= 0:
            return None
        return vec2(d.y, -d.z) / d.x

    @property
    def direction(self) -> vec3:
        return self.direction_X

    @property
    def rotation(self) -> vec3:
        return -eulerAngles(self.direction_Q)

    is_calibrating = False

    def calibrate(self, seconds=2):
        self.calibration_acumulator = vec3(0)
        self.calibration_acumulations = 0
        self.is_calibrating = time.time() + seconds

    def _set_calibration(self, gyro_offset=None):
        if not gyro_offset:
            c = vec3(1, self._ime_yz_coeff, self._ime_yz_coeff)
            gyro_offset = self.calibration_acumulator * c
            gyro_offset /= self.calibration_acumulations
            gyro_offset += vec3(
                    self._GYRO_OFFSET_X,
                    self._GYRO_OFFSET_Y,
                    self._GYRO_OFFSET_Z,
                )
        self.is_calibrating = False
        self.set_gyro_calibration(gyro_offset)

    def reset_orientation(self):
        self.direction_X = vec3(1, 0, 0)
        self.direction_Y = vec3(0, 1, 0)
        self.direction_Z = vec3(0, 0, 1)
        self.direction_Q = quat()

    @staticmethod
    def _gyro_update_hook(self):
        if self.is_calibrating:
            if self.is_calibrating < time.time():
                self._set_calibration()
            else:
                for xyz in self.gyro:
                    self.calibration_acumulator += xyz
                self.calibration_acumulations += 3

        for gx, gy, gz in self.gyro_in_rad:
            # TODO: find out why 1/86 works, and not 1/60 or 1/(60*30)
            rotation \
                = angleAxis(gx * (-1/86), self.direction_X) \
                * angleAxis(gy * (-1/86), self.direction_Y) \
                * angleAxis(gz * (-1/86), self.direction_Z)

            self.direction_X *= rotation
            self.direction_Y *= rotation
            self.direction_Z *= rotation
            self.direction_Q *= rotation
