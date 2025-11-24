from __future__ import annotations

from typing import SupportsFloat

import pyenki
import pyenki.viewer
from pyenki.adapters import Thymio2AsebaAdapter

from .utils import BodyColorPulse

DIR_LEFT = -1
DIR_L_LEFT = -2
DIR_RIGHT = 1
DIR_L_RIGHT = 2
DIR_LOST = 10
DIR_FRONT = 0
SPEED_LINE = 300
STATE_BLACK = 0
STATE_WHITE = 1


class ThymioLineFollowingBehavior:
    """
    Thymio line following behavior ported from the
    `firmware <https://github.com/Mobsya/aseba-target-thymio2/blob/master/mode.c#L687>`_,
    see ``static void tick_line(void)``.

    The original behavior ticks at 50Hz.
    It should be assigned to a Thymio, like in

    >>> thymio.control_step_callback = ThymioLineBehavior()
    """

    DT: float = 0.02

    def __init__(self, black: int = 300, white: int = 600) -> None:
        """
        Constructs a new instance.

        :param      black:  The ground sensors black threshold
        :param      white:  The ground sensors white threshold
        """
        self.s = [STATE_BLACK, STATE_BLACK]
        self.dir = DIR_FRONT
        self.bs_black_level: float = black
        self.bs_white_level: float = white
        self.body_color_pulse = BodyColorPulse()

    def __call__(self, thymio: pyenki.PhysicalObject,
                 dt: SupportsFloat) -> None:
        """Executes the behavior"""
        assert isinstance(thymio, pyenki.Thymio2)
        self.tick(Thymio2AsebaAdapter(thymio), float(dt))

    def tick(self, thymio: Thymio2AsebaAdapter, dt: float) -> None:
        steps = dt / self.DT
        stop_moving = False
        p = self.body_color_pulse.get(steps)
        thymio.call_leds_top(0, p, p)

        # Calibration feature
        buttons_state = thymio.thymio.buttons
        ground_delta = thymio.prox_ground_delta
        if buttons_state[1] and buttons_state[3]:
            self.bs_black_level = (ground_delta[0] + ground_delta[1]) / 2 + 150
            stop_moving = True

        if buttons_state[2] and buttons_state[4]:
            self.bs_white_level = (ground_delta[0] + ground_delta[1]) / 2
            if self.bs_white_level < 150:
                self.bs_white_level = 200
            self.bs_white_level -= 150
            stop_moving = False

        # if the user is trying to calibrate, then don't try to move
        if stop_moving:
            thymio.call_leds_circle(0, 0, 0, 0, 0, 0, 0, 0)
            thymio.motor_left_target = 0
            thymio.motor_right_target = 0
            return

        for i in range(2):
            if ground_delta[i] < self.bs_black_level:
                self.s[i] = STATE_BLACK
            if ground_delta[i] > self.bs_white_level:
                self.s[i] = STATE_WHITE

        if self.s[0] == STATE_BLACK and self.s[1] == STATE_BLACK:
            # Black line right under us
            self.dir = DIR_FRONT
        elif self.s[0] == STATE_WHITE and self.s[1] == STATE_BLACK:
            self.dir = DIR_RIGHT
        elif self.s[1] == STATE_WHITE and self.s[0] == STATE_BLACK:
            self.dir = DIR_LEFT
        else:
            # Lost
            if self.dir > 0:
                self.dir = DIR_L_RIGHT
            elif self.dir < 0:
                self.dir = DIR_L_LEFT
            else:
                self.dir = DIR_LOST

        if self.dir == DIR_FRONT:
            thymio.motor_left_target = SPEED_LINE
            thymio.motor_right_target = SPEED_LINE
            thymio.call_leds_circle(32, 0, 0, 0, 32, 0, 0, 0)
        elif self.dir == DIR_RIGHT:
            thymio.motor_left_target = SPEED_LINE
            thymio.motor_right_target = 0
            thymio.call_leds_circle(0, 32, 0, 32, 0, 0, 0, 0)
        elif self.dir == DIR_LEFT:
            thymio.motor_left_target = 0
            thymio.motor_right_target = SPEED_LINE
            thymio.call_leds_circle(0, 0, 0, 0, 0, 32, 0, 32)
        elif self.dir == DIR_L_LEFT:
            thymio.motor_left_target = -SPEED_LINE
            thymio.motor_right_target = SPEED_LINE
            thymio.call_leds_circle(0, 0, 0, 0, 0, 0, 32, 0)
        elif self.dir == DIR_L_RIGHT:
            thymio.motor_left_target = SPEED_LINE
            thymio.motor_right_target = -SPEED_LINE
            thymio.call_leds_circle(0, 0, 32, 0, 0, 0, 0, 0)
        elif self.dir == DIR_LOST:
            thymio.motor_left_target = SPEED_LINE
            thymio.motor_right_target = -SPEED_LINE
