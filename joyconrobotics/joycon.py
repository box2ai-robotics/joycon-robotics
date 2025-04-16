from .constants import JOYCON_VENDOR_ID, JOYCON_PRODUCT_IDS, JOYCON_SERIAL_HEAD
from .constants import JOYCON_L_PRODUCT_ID, JOYCON_R_PRODUCT_ID, JOYCON_REPORT_DEFAULT, JOYCON_SERIAL_SUPPORT
import hid
import time
import threading
from typing import Optional

# TODO: disconnect, power off sequence


class JoyCon:
    _INPUT_REPORT_SIZE = 49
    _INPUT_REPORT_PERIOD = 0.015
    _RUMBLE_DATA = b'\x00\x01\x40\x40\x00\x01\x40\x40'

    vendor_id  : int
    product_id : int
    serial     : Optional[str]
    simple_mode: bool
    color_body : (int, int, int)
    color_btn  : (int, int, int)

    def __init__(self, vendor_id: int, product_id: int, serial: str = None, simple_mode=False):
        if vendor_id != JOYCON_VENDOR_ID:
            raise ValueError(f'vendor_id is invalid: {vendor_id!r}')

        if product_id not in JOYCON_PRODUCT_IDS:
            raise ValueError(f'product_id is invalid: {product_id!r}')
        
        if serial[:12] not in JOYCON_SERIAL_HEAD:
            raise ValueError(f'serial is invalid: {serial!r}')

        self.vendor_id   = vendor_id
        self.product_id  = product_id
        self.serial      = serial
        self.simple_mode = simple_mode  # TODO: It's for reporting mode 0x3f
        
        if serial[:12] not in JOYCON_SERIAL_HEAD:
            self.calibrate_value = True
        else:
            self.calibrate_value = False
            
        # setup internal state
        self._input_hooks = []
        self._input_report = bytes(self._INPUT_REPORT_SIZE)
        self._packet_number = 0
        self.set_accel_calibration((0, 0, 0), (1, 1, 1))
        self.set_gyro_calibration((0, 0, 0), (1, 1, 1))

        # connect to joycon
        self.enable = True
        self._joycon_device = self._open(vendor_id, product_id, serial=None)
        self._read_joycon_data()
        self._setup_sensors()

        # start talking with the joycon in a daemon thread
        self._update_input_report_thread \
            = threading.Thread(target=self._update_input_report)
        self._update_input_report_thread.setDaemon(True)
        self._update_input_report_thread.start()

    def _open(self, vendor_id, product_id, serial):
        try:
            if hasattr(hid, "device"):  # hidapi
                _joycon_device = hid.device()
                _joycon_device.open(vendor_id, product_id, serial)
            elif hasattr(hid, "Device"):  # hid
                _joycon_device = hid.Device(vendor_id, product_id, serial)
            else:
                raise Exception("Implementation of hid is not recognized!")
        except IOError as e:
            raise IOError('joycon connect failed') from e
        return _joycon_device

    def _close(self):
        if hasattr(self, "_joycon_device"):
            self._joycon_device.close()
            self.enable = False
            del self._joycon_device

    def _read_input_report(self) -> bytes:
        return bytes(self._joycon_device.read(self._INPUT_REPORT_SIZE))

    def _write_output_report(self, command, subcommand, argument):
        # TODO: add documentation
        self._joycon_device.write(b''.join([
            command,
            self._packet_number.to_bytes(1, byteorder='little'),
            self._RUMBLE_DATA,
            subcommand,
            argument,
        ]))
        self._packet_number = (self._packet_number + 1) & 0xF

    def _send_subcmd_get_response(self, subcommand, argument) -> (bool, bytes):
        # TODO: handle subcmd when daemon is running
        self._write_output_report(b'\x01', subcommand, argument)

        report = self._read_input_report()
        while report[0] != 0x21:  # TODO, avoid this, await daemon instead
            report = self._read_input_report()

        # TODO, remove, see the todo above
        # assert report[1:2] != subcommand, "THREAD carefully"

        # TODO: determine if the cut bytes are worth anything

        return report[13] & 0x80, report[13:]  # (ack, data)

    def _spi_flash_read(self, address, size) -> bytes:
        assert size <= 0x1d
        argument = address.to_bytes(4, "little") + size.to_bytes(1, "little")
        
        ack, report = self._send_subcmd_get_response(b'\x10', argument)
        if not ack:
            raise IOError("After SPI read @ {address:#06x}: got NACK")
        
        out = report[7:size+7]
        
        # if out == JOYCON_REPORT_DEFAULT and report[:3] != JOYCON_COLOR_SUPPORT:
        #     print(f'{report[:3]=}')
        #     raise IOError("Something else than the expected ACK was recieved!")
        
        return out

    def _update_input_report(self):  # daemon thread
        while True and self.enable == True :
            report = self._read_input_report()
            # TODO, handle input reports of type 0x21 and 0x3f
            while report[0] != 0x30:
                report = self._read_input_report()

            self._input_report = report

            for callback in self._input_hooks:
                callback(self)

    def _read_joycon_data(self):
        color_data = self._spi_flash_read(0x6050, 6)
        
        # TODO: use this
        # stick_cal_addr = 0x8012 if self.is_left else 0x801D
        # stick_cal  = self._spi_flash_read(stick_cal_addr, 8)

        # user IME data
        if self._spi_flash_read(0x8026, 2) == b"\xB2\xA1":
            # print(f"Calibrate {self.serial} IME with user data")
            imu_cal = self._spi_flash_read(0x8028, 24)

        # factory IME data
        else:
            # print(f"Calibrate {self.serial} IME with factory data")
            imu_cal = self._spi_flash_read(0x6020, 24)

        self.color_body = tuple(color_data[:3])
        self.color_btn  = tuple(color_data[3:])

        self.set_accel_calibration((
                self._to_int16le_from_2bytes(imu_cal[ 0], imu_cal[ 1]),
                self._to_int16le_from_2bytes(imu_cal[ 2], imu_cal[ 3]),
                self._to_int16le_from_2bytes(imu_cal[ 4], imu_cal[ 5]),
            ), (
                self._to_int16le_from_2bytes(imu_cal[ 6], imu_cal[ 7]),
                self._to_int16le_from_2bytes(imu_cal[ 8], imu_cal[ 9]),
                self._to_int16le_from_2bytes(imu_cal[10], imu_cal[11]),
            )
        )
        self.set_gyro_calibration((
                self._to_int16le_from_2bytes(imu_cal[12], imu_cal[13]),
                self._to_int16le_from_2bytes(imu_cal[14], imu_cal[15]),
                self._to_int16le_from_2bytes(imu_cal[16], imu_cal[17]),
            ), (
                self._to_int16le_from_2bytes(imu_cal[18], imu_cal[19]),
                self._to_int16le_from_2bytes(imu_cal[20], imu_cal[21]),
                self._to_int16le_from_2bytes(imu_cal[22], imu_cal[23]),
            )
        )

    def _setup_sensors(self):
        # Enable 6 axis sensors
        self._write_output_report(b'\x01', b'\x40', b'\x01')
        # It needs delta time to update the setting
        time.sleep(0.02)
        # Change format of input report
        self._write_output_report(b'\x01', b'\x03', b'\x30')

    @staticmethod
    def _to_int16le_from_2bytes(hbytebe, lbytebe):
        uint16le = (lbytebe << 8) | hbytebe
        int16le = uint16le if uint16le < 32768 else uint16le - 65536
        return int16le

    def _get_nbit_from_input_report(self, offset_byte, offset_bit, nbit):
        byte = self._input_report[offset_byte]
        return (byte >> offset_bit) & ((1 << nbit) - 1)

    def __del__(self):
        self._close()

    def set_gyro_calibration(self, offset_xyz=None, coeff_xyz=None):
        if offset_xyz:
            self._GYRO_OFFSET_X, \
            self._GYRO_OFFSET_Y, \
            self._GYRO_OFFSET_Z = offset_xyz
        if coeff_xyz:
            cx, cy, cz = coeff_xyz
            self._GYRO_COEFF_X = 0x343b / cx if (cx != 0x343b and cx != 0 and self.calibrate_value) else 1
            self._GYRO_COEFF_Y = 0x343b / cy if (cy != 0x343b and cy != 0 and self.calibrate_value) else 1
            self._GYRO_COEFF_Z = 0x343b / cz if (cz != 0x343b and cz != 0 and self.calibrate_value) else 1

    def set_accel_calibration(self, offset_xyz=None, coeff_xyz=None):
        if offset_xyz:
            self._ACCEL_OFFSET_X, \
            self._ACCEL_OFFSET_Y, \
            self._ACCEL_OFFSET_Z = offset_xyz
        if coeff_xyz:
            cx, cy, cz = coeff_xyz
            self._ACCEL_COEFF_X = 0x4000 / cx if (cx != 0x4000 and cx != 0) else 1
            self._ACCEL_COEFF_Y = 0x4000 / cy if (cy != 0x4000 and cy != 0) else 1
            self._ACCEL_COEFF_Z = 0x4000 / cz if (cz != 0x4000 and cz != 0) else 1

    def register_update_hook(self, callback):
        self._input_hooks.append(callback)
        return callback  # this makes it so you could use it as a decorator

    def is_left(self):
        return self.product_id == JOYCON_L_PRODUCT_ID

    def is_right(self):
        return self.product_id == JOYCON_R_PRODUCT_ID

    def get_battery_charging(self):
        return self._get_nbit_from_input_report(2, 4, 1)

    def get_battery_level(self):
        return self._get_nbit_from_input_report(2, 5, 3)

    def get_button_y(self):
        return self._get_nbit_from_input_report(3, 0, 1)

    def get_button_x(self):
        return self._get_nbit_from_input_report(3, 1, 1)

    def get_button_b(self):
        return self._get_nbit_from_input_report(3, 2, 1)

    def get_button_a(self):
        return self._get_nbit_from_input_report(3, 3, 1)

    def get_button_right_sr(self):
        return self._get_nbit_from_input_report(3, 4, 1)

    def get_button_right_sl(self):
        return self._get_nbit_from_input_report(3, 5, 1)

    def get_button_r(self):
        return self._get_nbit_from_input_report(3, 6, 1)

    def get_button_zr(self):
        return self._get_nbit_from_input_report(3, 7, 1)

    def get_button_minus(self):
        return self._get_nbit_from_input_report(4, 0, 1)

    def get_button_plus(self):
        return self._get_nbit_from_input_report(4, 1, 1)

    def get_button_r_stick(self):
        return self._get_nbit_from_input_report(4, 2, 1)

    def get_button_l_stick(self):
        return self._get_nbit_from_input_report(4, 3, 1)

    def get_button_home(self):
        return self._get_nbit_from_input_report(4, 4, 1)

    def get_button_capture(self):
        return self._get_nbit_from_input_report(4, 5, 1)

    def get_button_charging_grip(self):
        return self._get_nbit_from_input_report(4, 7, 1)

    def get_button_down(self):
        return self._get_nbit_from_input_report(5, 0, 1)

    def get_button_up(self):
        return self._get_nbit_from_input_report(5, 1, 1)

    def get_button_right(self):
        return self._get_nbit_from_input_report(5, 2, 1)

    def get_button_left(self):
        return self._get_nbit_from_input_report(5, 3, 1)

    def get_button_left_sr(self):
        return self._get_nbit_from_input_report(5, 4, 1)

    def get_button_left_sl(self):
        return self._get_nbit_from_input_report(5, 5, 1)

    def get_button_l(self):
        return self._get_nbit_from_input_report(5, 6, 1)

    def get_button_zl(self):
        return self._get_nbit_from_input_report(5, 7, 1)

    def get_stick_left_horizontal(self):
        return self._get_nbit_from_input_report(6, 0, 8) \
            | (self._get_nbit_from_input_report(7, 0, 4) << 8)

    def get_stick_left_vertical(self):
        return self._get_nbit_from_input_report(7, 4, 4) \
            | (self._get_nbit_from_input_report(8, 0, 8) << 4)

    def get_stick_right_horizontal(self):
        return self._get_nbit_from_input_report(9, 0, 8) \
            | (self._get_nbit_from_input_report(10, 0, 4) << 8)

    def get_stick_right_vertical(self):
        return self._get_nbit_from_input_report(10, 4, 4) \
            | (self._get_nbit_from_input_report(11, 0, 8) << 4)

    def get_accel_x(self, sample_idx=0):
        if sample_idx not in (0, 1, 2):
            raise IndexError('sample_idx should be between 0 and 2')
        data = self._to_int16le_from_2bytes(
            self._input_report[13 + sample_idx * 12],
            self._input_report[14 + sample_idx * 12])
        return (data - self._ACCEL_OFFSET_X) * self._ACCEL_COEFF_X

    def get_accel_y(self, sample_idx=0):
        if sample_idx not in (0, 1, 2):
            raise IndexError('sample_idx should be between 0 and 2')
        data = self._to_int16le_from_2bytes(
            self._input_report[15 + sample_idx * 12],
            self._input_report[16 + sample_idx * 12])
        return (data - self._ACCEL_OFFSET_Y) * self._ACCEL_COEFF_Y

    def get_accel_z(self, sample_idx=0):
        if sample_idx not in (0, 1, 2):
            raise IndexError('sample_idx should be between 0 and 2')
        data = self._to_int16le_from_2bytes(
            self._input_report[17 + sample_idx * 12],
            self._input_report[18 + sample_idx * 12])
        return (data - self._ACCEL_OFFSET_Z) * self._ACCEL_COEFF_Z

    def get_gyro_x(self, sample_idx=0):
        if sample_idx not in (0, 1, 2):
            raise IndexError('sample_idx should be between 0 and 2')
        data = self._to_int16le_from_2bytes(
            self._input_report[19 + sample_idx * 12],
            self._input_report[20 + sample_idx * 12])
        return (data - self._GYRO_OFFSET_X) * self._GYRO_COEFF_X

    def get_gyro_y(self, sample_idx=0):
        if sample_idx not in (0, 1, 2):
            raise IndexError('sample_idx should be between 0 and 2')
        data = self._to_int16le_from_2bytes(
            self._input_report[21 + sample_idx * 12],
            self._input_report[22 + sample_idx * 12])
        return (data - self._GYRO_OFFSET_Y) * self._GYRO_COEFF_Y

    def get_gyro_z(self, sample_idx=0):
        if sample_idx not in (0, 1, 2):
            raise IndexError('sample_idx should be between 0 and 2')
        data = self._to_int16le_from_2bytes(
            self._input_report[23 + sample_idx * 12],
            self._input_report[24 + sample_idx * 12])
        return (data - self._GYRO_OFFSET_Z) * self._GYRO_COEFF_Z

    def get_status(self) -> dict:
        return {
            "battery": {
                "charging": self.get_battery_charging(),
                "level": self.get_battery_level(),
            },
            "buttons": {
                "right": {
                    "y": self.get_button_y(),
                    "x": self.get_button_x(),
                    "b": self.get_button_b(),
                    "a": self.get_button_a(),
                    "sr": self.get_button_right_sr(),
                    "sl": self.get_button_right_sl(),
                    "r": self.get_button_r(),
                    "zr": self.get_button_zr(),
                },
                "shared": {
                    "minus": self.get_button_minus(),
                    "plus": self.get_button_plus(),
                    "r-stick": self.get_button_r_stick(),
                    "l-stick": self.get_button_l_stick(),
                    "home": self.get_button_home(),
                    "capture": self.get_button_capture(),
                    "charging-grip": self.get_button_charging_grip(),
                },
                "left": {
                    "down": self.get_button_down(),
                    "up": self.get_button_up(),
                    "right": self.get_button_right(),
                    "left": self.get_button_left(),
                    "sr": self.get_button_left_sr(),
                    "sl": self.get_button_left_sl(),
                    "l": self.get_button_l(),
                    "zl": self.get_button_zl(),
                }
            },
            "analog-sticks": {
                "left": {
                    "horizontal": self.get_stick_left_horizontal(),
                    "vertical": self.get_stick_left_vertical(),
                },
                "right": {
                    "horizontal": self.get_stick_right_horizontal(),
                    "vertical": self.get_stick_right_vertical(),
                },
            },
            "accel": {
                "x": self.get_accel_x(),
                "y": self.get_accel_y(),
                "z": self.get_accel_z(),
            },
            "gyro": {
                "x": self.get_gyro_x(),
                "y": self.get_gyro_y(),
                "z": self.get_gyro_z(),
            },
        }

    def set_player_lamp_on(self, on_pattern: int):
        self._write_output_report(
            b'\x01', b'\x30',
            (on_pattern & 0xF).to_bytes(1, byteorder='little'))

    def set_player_lamp_flashing(self, flashing_pattern: int):
        self._write_output_report(
            b'\x01', b'\x30',
            ((flashing_pattern & 0xF) << 4).to_bytes(1, byteorder='little'))

    def set_player_lamp(self, pattern: int):
        self._write_output_report(
            b'\x01', b'\x30',
            pattern.to_bytes(1, byteorder='little'))

    def disconnect_device(self):
        self._write_output_report(b'\x01', b'\x06', b'\x00')


if __name__ == '__main__':
    import pyjoycon.device as d
    ids = d.get_L_id() if None not in d.get_L_id() else d.get_R_id()

    if None not in ids:
        joycon = JoyCon(*ids)
        lamp_pattern = 0
        while True:
            print(joycon.get_status())
            joycon.set_player_lamp_on(lamp_pattern)
            lamp_pattern = (lamp_pattern + 1) & 0xf
            time.sleep(0.2)
