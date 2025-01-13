from .wrappers import PythonicJoyCon


class ButtonEventJoyCon(PythonicJoyCon):
    def __init__(self, *args, track_sticks=False, **kwargs):
        super().__init__(*args, **kwargs)

        self._events_buffer = []  # TODO: perhaps use a deque instead?

        self._event_handlers = {}
        self._event_track_sticks = track_sticks

        self._previous_stick_l_btn = 0
        self._previous_stick_r_btn = 0
        self._previous_stick_r  = self._previous_stick_l  = (0, 0)
        self._previous_r        = self._previous_l        = 0
        self._previous_zr       = self._previous_zl       = 0
        self._previous_plus     = self._previous_minus    = 0
        self._previous_a        = self._previous_right    = 0
        self._previous_b        = self._previous_down     = 0
        self._previous_x        = self._previous_up       = 0
        self._previous_y        = self._previous_left     = 0
        self._previous_home     = self._previous_capture  = 0
        self._previous_right_sr = self._previous_left_sr  = 0
        self._previous_right_sl = self._previous_left_sl  = 0

        if self.is_left():
            self.register_update_hook(self._event_tracking_update_hook_left)
        else:
            self.register_update_hook(self._event_tracking_update_hook_right)

    def joycon_button_event(self, button, state):  # overridable
        self._events_buffer.append((button, state))

    def events(self):
        while self._events_buffer:
            yield self._events_buffer.pop(0)

    @staticmethod
    def _event_tracking_update_hook_right(self):
        if self._event_track_sticks:
            pressed = self.stick_r_btn
            if self._previous_stick_r_btn != pressed:
                self._previous_stick_r_btn = pressed
                self.joycon_button_event("stick_r_btn", pressed)
        pressed = self.r
        if self._previous_r != pressed:
            self._previous_r = pressed
            self.joycon_button_event("r", pressed)
        pressed = self.zr
        if self._previous_zr != pressed:
            self._previous_zr = pressed
            self.joycon_button_event("zr", pressed)
        pressed = self.plus
        if self._previous_plus != pressed:
            self._previous_plus = pressed
            self.joycon_button_event("plus", pressed)
        pressed = self.a
        if self._previous_a != pressed:
            self._previous_a = pressed
            self.joycon_button_event("a", pressed)
        pressed = self.b
        if self._previous_b != pressed:
            self._previous_b = pressed
            self.joycon_button_event("b", pressed)
        pressed = self.x
        if self._previous_x != pressed:
            self._previous_x = pressed
            self.joycon_button_event("x", pressed)
        pressed = self.y
        if self._previous_y != pressed:
            self._previous_y = pressed
            self.joycon_button_event("y", pressed)
        pressed = self.home
        if self._previous_home != pressed:
            self._previous_home = pressed
            self.joycon_button_event("home", pressed)
        pressed = self.right_sr
        if self._previous_right_sr != pressed:
            self._previous_right_sr = pressed
            self.joycon_button_event("right_sr", pressed)
        pressed = self.right_sl
        if self._previous_right_sl != pressed:
            self._previous_right_sl = pressed
            self.joycon_button_event("right_sl", pressed)

    @staticmethod
    def _event_tracking_update_hook_left(self):
        if self._event_track_sticks:
            pressed = self.stick_l_btn
            if self._previous_stick_l_btn != pressed:
                self._previous_stick_l_btn = pressed
                self.joycon_button_event("stick_l_btn", pressed)
        pressed = self.l
        if self._previous_l != pressed:
            self._previous_l = pressed
            self.joycon_button_event("l", pressed)
        pressed = self.zl
        if self._previous_zl != pressed:
            self._previous_zl = pressed
            self.joycon_button_event("zl", pressed)
        pressed = self.minus
        if self._previous_minus != pressed:
            self._previous_minus = pressed
            self.joycon_button_event("minus", pressed)
        pressed = self.up
        if self._previous_up != pressed:
            self._previous_up = pressed
            self.joycon_button_event("up", pressed)
        pressed = self.down
        if self._previous_down != pressed:
            self._previous_down = pressed
            self.joycon_button_event("down", pressed)
        pressed = self.left
        if self._previous_left != pressed:
            self._previous_left = pressed
            self.joycon_button_event("left", pressed)
        pressed = self.right
        if self._previous_right != pressed:
            self._previous_right = pressed
            self.joycon_button_event("right", pressed)
        pressed = self.capture
        if self._previous_capture != pressed:
            self._previous_capture = pressed
            self.joycon_button_event("capture", pressed)
        pressed = self.left_sr
        if self._previous_left_sr != pressed:
            self._previous_left_sr = pressed
            self.joycon_button_event("left_sr", pressed)
        pressed = self.left_sl
        if self._previous_left_sl != pressed:
            self._previous_left_sl = pressed
            self.joycon_button_event("left_sl", pressed)
