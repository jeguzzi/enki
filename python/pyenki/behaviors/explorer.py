from __future__ import annotations

import math
from typing import SupportsFloat

from .. import PhysicalObject, Thymio2
from ..adapters import Thymio2AsebaAdapter, umod
from .utils import BodyColorPulse, check_grounds, clip_speed


class ThymioExplorerBehavior:
    """
    Thymio Explorer behavior ported from the
    `firmware <https://github.com/Mobsya/aseba-target-thymio2/blob/master/mode.c#L416>`_,
    see ``static void tick_explorer(void)``.

    It is also very similar to the `aseba implementation <https://aseba.wdfiles.com/local--files/en:thymiobehaviourexplorer/explorateurV6.aesl>`_.

    The original behavior ticks at 50Hz.
    It should be assigned to a Thymio, like in

    >>> thymio.control_step_callback = ThymioExplorerController()
    """
    DT: float = 0.02

    def __init__(self, prox_value: int = 1000):
        """
        Constructs a new instance.

        :param      prox_value:  the threshold to move back.
            The original implementation uses a (very low) 222,
            which is too reactive in simulation.
        """
        self.led_state = 0
        self.speed: float = 150
        self.body_color_pulse = BodyColorPulse()
        self.prox_value = 9 * prox_value

    def __call__(self, thymio: PhysicalObject, dt: SupportsFloat) -> None:
        """Executes the behavior"""
        assert isinstance(thymio, Thymio2)
        self.tick(Thymio2AsebaAdapter(thymio), float(dt))

    def tick(self, thymio: Thymio2AsebaAdapter, dt: float) -> None:
        steps = dt / self.DT
        p = self.body_color_pulse.get(steps)
        # print(p)
        thymio.call_leds_top(p, p, 0)

        self.led_state = umod(math.floor(self.led_state + 2 * steps), 8)
        led_state = math.floor(self.led_state)
        fixed = led_state // 32
        leds = [0] * 8
        leds[fixed] = 32
        leds[(fixed - 1) & 0x7] = 32 - (led_state & 0x1F)
        leds[(fixed + 1) & 0x7] = led_state & 0x1F
        thymio.call_leds_circle(*leds)

        if thymio.button_forward:
            self.speed = min(500, self.speed + 50 * steps)
        if thymio.button_backward:
            self.speed = max(-300, self.speed - 50 * steps)

        prox_horizontal = thymio.prox_horizontal

        if self.speed >= 0:
            temp1 = sum(k * v for k, v in zip(
                [1, 2, 3, 2, 1, 0, 0], prox_horizontal, strict=True))
            temp2 = sum(k * v for k, v in zip(
                [-4, -3, 0, 3, 4, 0, 0], prox_horizontal, strict=True))
            thymio.motor_left_target = clip_speed(self.speed -
                                                  (temp1 + temp2) *
                                                  self.speed / 16000)
            thymio.motor_right_target = clip_speed(self.speed -
                                                   (temp1 - temp2) *
                                                   self.speed / 16000)
        else:
            thymio.motor_left_target = clip_speed(self.speed -
                                                  prox_horizontal[6] *
                                                  self.speed / 300)
            thymio.motor_right_target = clip_speed(self.speed -
                                                   prox_horizontal[5] *
                                                   self.speed / 300)

        check_grounds(thymio)
