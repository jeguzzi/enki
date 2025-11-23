from __future__ import annotations

import math
from typing import cast

from ..adapters import Thymio2AsebaAdapter


def leds_set_body_rgb(thymio: Thymio2AsebaAdapter, r: int, g: int,
                      b: int) -> None:
    thymio.call_leds_top(r, g, b)
    thymio.call_leds_bottom_left(r, g, b)
    thymio.call_leds_bottom_right(r, g, b)


def clip_speed(value: float, max_value: int = 600) -> int:
    return max(-max_value, min(max_value, math.floor(value)))


def check_grounds(thymio: Thymio2AsebaAdapter, max_value: int = 130) -> None:
    prox_ground_delta = thymio.prox_ground_delta
    if prox_ground_delta[0] < max_value or prox_ground_delta[1] < max_value:
        thymio.motor_left_target = 0
        thymio.motor_right_target = 0
        thymio.call_leds_bottom_left(32, 0, 0)
        thymio.call_leds_bottom_right(32, 0, 0)
    else:
        thymio.call_leds_bottom_left(0, 0, 0)
        thymio.call_leds_bottom_right(0, 0, 0)


class BodyColorPulse:

    def __init__(self) -> None:
        self.led_pulse: float = 0

    def get(self, steps: float) -> int:
        self.led_pulse += steps
        if self.led_pulse > 0:
            ret = self.led_pulse
            if self.led_pulse > 40:
                self.led_pulse = -128
        else:
            ret = -self.led_pulse / 4
        return math.floor(ret)


def _rainbow_get(i: int) -> int:
    if i < 32:
        return i
    if i < 64:
        return 64 - i
    return 0


class Rainbow:

    def __init__(self) -> None:
        self.led_i: float = 0

    def get(self, steps: float) -> tuple[int, int, int]:
        self.led_i = self.led_i + steps
        if self.led_i > 96:
            self.led_i = 0
        r = self.led_i
        g = self.led_i + 32
        if g > 96:
            g -= 96
        b = self.led_i + 64
        if b > 96:
            b -= 96
        return cast('tuple[int, int, int]',
                    tuple(_rainbow_get(math.floor(x)) for x in (r, g, b)))
