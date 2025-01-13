from .joycon import JoyCon


# Preferably, this class gets merged into the
# parent class if approved by the original author
class PythonicJoyCon(JoyCon):
    """
    A wrapper class for the JoyCon parent class.
    This creates a more pythonic interface by
     *  using properties instead of requiring java-style getters and setters,
     *  bundles related xy/xyz data in tuples
     *  bundles the multiple measurements of the
        gyroscope and accelerometer into a list
     *  Adds the option to invert the y and z axis of the left joycon
        to make it match the right joycon. This is enabled by default
    """

    def __init__(self, *a, invert_left_ime_yz=True, **kw):
        super().__init__(*a, **kw)
        self._ime_yz_coeff = -1 if invert_left_ime_yz and self.is_left() else 1

    is_charging   = property(JoyCon.get_battery_charging)
    battery_level = property(JoyCon.get_battery_level)

    r             = property(JoyCon.get_button_r)
    zr            = property(JoyCon.get_button_zr)
    plus          = property(JoyCon.get_button_plus)
    a             = property(JoyCon.get_button_a)
    b             = property(JoyCon.get_button_b)
    x             = property(JoyCon.get_button_x)
    y             = property(JoyCon.get_button_y)
    stick_r_btn   = property(JoyCon.get_button_r_stick)
    home          = property(JoyCon.get_button_home)
    right_sr      = property(JoyCon.get_button_right_sr)
    right_sl      = property(JoyCon.get_button_right_sl)

    l             = property(JoyCon.get_button_l)  # noqa: E741
    zl            = property(JoyCon.get_button_zl)
    minus         = property(JoyCon.get_button_minus)
    stick_l_btn   = property(JoyCon.get_button_l_stick)
    up            = property(JoyCon.get_button_up)
    down          = property(JoyCon.get_button_down)
    left          = property(JoyCon.get_button_left)
    right         = property(JoyCon.get_button_right)
    capture       = property(JoyCon.get_button_capture)
    left_sr       = property(JoyCon.get_button_left_sr)
    left_sl       = property(JoyCon.get_button_left_sl)

    set_led_on       = JoyCon.set_player_lamp_on
    set_led_flashing = JoyCon.set_player_lamp_flashing
    set_led          = JoyCon.set_player_lamp
    disconnect       = JoyCon.disconnect_device

    @property
    def stick_l(self):
        return (
            self.get_stick_left_horizontal(),
            self.get_stick_left_vertical(),
        )

    @property
    def stick_r(self):
        return (
            self.get_stick_right_horizontal(),
            self.get_stick_right_vertical(),
        )

    @property
    def accel(self):
        c = self._ime_yz_coeff
        return [
            (
                self.get_accel_x(i),
                self.get_accel_y(i) * c,
                self.get_accel_z(i) * c,
            )
            for i in range(3)
        ]

    @property
    def accel_in_g(self):
        c = 4.0 / 0x4000
        c2 = c * self._ime_yz_coeff
        return [
            (
                self.get_accel_x(i) * c,
                self.get_accel_y(i) * c2,
                self.get_accel_z(i) * c2,
            )
            for i in range(3)
        ]

    @property
    def gyro(self):
        c = self._ime_yz_coeff
        return [
            (
                self.get_gyro_x(i),
                self.get_gyro_y(i) * c,
                self.get_gyro_z(i) * c,
            )
            for i in range(3)
        ]

    @property
    def gyro_in_deg(self):
        c = 0.06103
        c2 = c * self._ime_yz_coeff
        return [
            (
                self.get_gyro_x(i) * c,
                self.get_gyro_y(i) * c2,
                self.get_gyro_z(i) * c2,
            )
            for i in range(3)
        ]

    @property
    def gyro_in_rad(self):
        c = 0.0001694 * 3.1415926536
        c2 = c * self._ime_yz_coeff
        return [
            (
                self.get_gyro_x(i) * c,
                self.get_gyro_y(i) * c2,
                self.get_gyro_z(i) * c2,
            )
            for i in range(3)
        ]

    @property
    def gyro_in_rot(self):
        c = 0.0001694
        c2 = c * self._ime_yz_coeff
        return [
            (
                self.get_gyro_x(i) * c,
                self.get_gyro_y(i) * c2,
                self.get_gyro_z(i) * c2,
            )
            for i in range(3)
        ]
