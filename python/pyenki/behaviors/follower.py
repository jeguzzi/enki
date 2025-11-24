from __future__ import annotations

import math
from operator import itemgetter
from typing import SupportsFloat

import pyenki
import pyenki.viewer
from pyenki.adapters import Thymio2AsebaAdapter, umod

from .utils import (BodyColorPulse, Rainbow, check_grounds, clip_speed,
                    leds_set_body_rgb)


def find_max(values: list[int]) -> tuple[int, int]:
    return sorted(enumerate(values), key=itemgetter(1), reverse=True)[0]


class ThymioFollowerBehavior:
    """
    Thymio Follower behavior ported from the
    `firmware <https://github.com/Mobsya/aseba-target-thymio2/blob/master/mode.c#L272>`_,
    see ``static void tick_follow(void)``.

    The original behavior ticks at 50Hz.
    It assumes that other "friendly" robots have enabled proximity communication.
    It should be assigned to a Thymio, like in

    >>> thymio.prox_comm_enabled = True
    >>> thymio.control_step_callback = ThymioExplorerController()
    """

    DETECT: int = 500
    DT: float = 0.02

    def __init__(self) -> None:
        self.sound_done = 0
        self.led_delta = 1
        self.led_state: float = 0
        self.speed = 300
        self.does_see_friend = 0
        self.body_color_pulse = BodyColorPulse()
        self.rainbow = Rainbow()

    def __call__(self, thymio: pyenki.PhysicalObject,
                 dt: SupportsFloat) -> None:
        """Executes the behavior"""
        assert isinstance(thymio, pyenki.Thymio2)
        self.tick(Thymio2AsebaAdapter(thymio), float(dt))

    def tick(self, thymio: Thymio2AsebaAdapter, dt: float) -> None:
        steps = dt / self.DT
        speed_l: float = 0
        prox_horizontal = thymio.prox_horizontal
        mi, max_prox = find_max(prox_horizontal[:5])
        speed_diff = (2 - mi) * self.speed / 2
        if max_prox > 4000:
            speed_l = -self.speed
        elif max_prox > 3500:
            speed_l = (3500 - max_prox) / 2
        elif max_prox < 2000:
            speed_l = self.speed
        elif max_prox < 3000:
            speed_l = 300 - (max_prox - 1000) / 7
        speed_l = clip_speed(speed_l, self.speed)

        if max_prox < self.DETECT:
            if self.does_see_friend:
                thymio.motor_left_target = self.speed
                thymio.motor_right_target = self.speed
            else:
                thymio.motor_left_target = 0
                thymio.motor_right_target = 0
        else:
            thymio.motor_right_target = math.floor(speed_diff + speed_l)
            thymio.motor_left_target = math.floor(speed_l - speed_diff)

        if self.does_see_friend > 0 and self.sound_done:
            rgb = self.rainbow.get(steps)
            leds_set_body_rgb(thymio, rgb[0], rgb[1], rgb[2])
        else:
            leds_set_body_rgb(thymio, 0, self.body_color_pulse.get(steps), 0)

        if self.does_see_friend:
            self.led_state += self.led_delta * steps
            if self.led_state >= 31:
                self.led_delta = -1
            elif self.led_state == 0:
                self.led_delta = 1
            led_state = umod(math.floor(self.led_state), 8)
            thymio.call_leds_circle(0, led_state >> 4, led_state >> 3,
                                    led_state, 32, led_state, led_state >> 3,
                                    led_state >> 4)
        else:
            thymio.call_leds_circle(0, 0, 0, 32, 32, 32, 0, 0)

        if thymio.button_forward:
            self.speed = min(500, math.floor(self.speed + 50 * steps))
        if thymio.button_backward:
            self.speed = max(-300, math.floor(self.speed - 50 * steps))

        if speed_diff == 0 and speed_l == 0 and self.sound_done == 0 and max_prox > self.DETECT:
            self.sound_done = 1
        if speed_diff != 0 or max_prox < self.DETECT:
            self.sound_done = 0

        check_grounds(thymio)

        if self.does_see_friend:
            self.does_see_friend -= 1

        for intensities, rx in zip(thymio.prox_comm_rx__intensities,
                                   thymio.prox_comm_rx,
                                   strict=True):
            self.does_see_friend = 0
            mi, max_prox = find_max(intensities)
            if max_prox > 3000:
                thymio.prox_comm_tx = mi
                if 0 < rx < 4:
                    if mi == 2:
                        self.does_see_friend = 6
