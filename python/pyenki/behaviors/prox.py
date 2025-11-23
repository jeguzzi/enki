from __future__ import annotations

from typing import SupportsFloat

import numpy as np

from .. import PhysicalObject, Thymio2
from ..adapters import Thymio2AsebaAdapter


class ThymioLEDProxBehavior:
    """
    Thymio LED prox behavior ported from the
    `firmware <https://github.com/Mobsya/aseba-target-thymio2/blob/master/behavior.c#L243>`_,
    see ``static void behavior_leds_prox(void)``.

    The original behavior ticks at 50Hz.
    It should be assigned to a Thymio, like in

    >>> thymio.control_step_callback = ThymioLEDProxBehavior()
    """

    def __init__(self) -> None:
        self.max = np.array(
            [4000, 4000, 4000, 4000, 4000, 4000, 4000, 900, 900])
        self.min = np.array([1200, 1200, 1200, 1200, 1200, 1200, 1200, 0, 0])

    def __call__(self, thymio: PhysicalObject, dt: SupportsFloat) -> None:
        """Executes the behavior"""
        assert isinstance(thymio, Thymio2)
        self.tick(Thymio2AsebaAdapter(thymio), float(dt))

    def tick(self, thymio: Thymio2AsebaAdapter, dt: float) -> None:
        values = np.asarray(thymio.prox_horizontal + thymio.prox_ground_delta)
        not_zero = values > 0
        self.max = np.maximum(values, self.max)
        self.min[not_zero] = np.minimum(values[not_zero], self.min[not_zero])
        bs = np.round(32 * np.clip(
            (values - self.min) / (self.max - self.min), 0, 1))
        thymio.call_leds_prox_h(*bs[:7])
        # TODO: uncomment when we implement call_leds_prox_v
        # thymio.call_leds_prox_v(*bs[-2:])
