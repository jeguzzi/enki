from __future__ import annotations

from typing import SupportsFloat

from .. import PhysicalObject, Thymio2
from ..adapters import Thymio2AsebaAdapter
from .utils import BodyColorPulse, check_grounds, clip_speed, leds_set_body_rgb


class ThymioAccBehavior:
    """
    Thymio Acc behavior ported from the
    `firmware <https://github.com/Mobsya/aseba-target-thymio2/blob/master/mode.c#L495>`_,
    see ``static void tick_acc(void)``.

    The original behavior ticks at 50Hz.
    It should be assigned to a Thymio, like in

    >>> thymio.control_step_callback = ThymioAccBehavior()
    """

    ACC_OBSTACLE: int = 1000
    ACC_FREE_FALL_TRESH: int = 14
    DT: float = 0.02

    def __init__(self) -> None:
        self.acc = 32.0
        self.counter: float = 0
        self.body_color_pulse = BodyColorPulse()

    def __call__(self, thymio: PhysicalObject, dt: SupportsFloat) -> None:
        """Executes the behavior"""
        assert isinstance(thymio, Thymio2)
        self.tick(Thymio2AsebaAdapter(thymio), float(dt))

    def tick(self, thymio: Thymio2AsebaAdapter, dt: float) -> None:
        acc = thymio.acc
        steps = dt / self.DT
        self.acc = (self.acc * 3 + abs(acc[0]) + abs(acc[1]) + abs(acc[2])) / 4
        if self.acc < self.ACC_FREE_FALL_TRESH:
            self.counter += steps
            if self.counter > 5:
                if self.counter >= 10:
                    self.counter = 0
                thymio.call_leds_top(32, 0, 0)
            else:
                thymio.call_leds_top(0, 0, 0)
        else:
            leds_set_body_rgb(thymio, self.body_color_pulse.get(steps), 0, 0)

        prox = thymio.prox_horizontal
        prox_ground_delta = thymio.prox_ground_delta
        if all(prox[i] > self.ACC_OBSTACLE
               for i in (1, 2, 3, 5, 6)) and all(x > 130
                                                 for x in prox_ground_delta):
            thymio.motor_left_target = 0
            thymio.motor_right_target = 0
        elif any(prox[i] > self.ACC_OBSTACLE for i in (0, 1, 2, 3, 4)):
            temp = prox[0] / 5 + prox[1] / 4 + prox[2] / 4 + prox[
                3] / 4 + prox[4] / 5
            temp2 = prox[0] / 6 + prox[1] / 5 - prox[3] / 5 + prox[4] / 6

            thymio.motor_left_target = clip_speed(-(temp + temp2))
            thymio.motor_right_target = clip_speed(temp2 - temp)
        elif any(prox[i] > self.ACC_OBSTACLE for i in (5, 6)):
            thymio.motor_left_target = clip_speed(prox[5] / 4)
            thymio.motor_right_target = clip_speed(prox[6] / 4)
        else:
            thymio.motor_left_target = 0
            thymio.motor_right_target = 0

        check_grounds(thymio)
