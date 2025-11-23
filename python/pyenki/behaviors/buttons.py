from __future__ import annotations

from typing import SupportsFloat

import numpy as np

from .. import PhysicalObject, Thymio2
from ..adapters import Thymio2AsebaAdapter


class ThymioLEDButtonsBehavior:
    """
    Thymio LED prox behavior ported from the
    `firmware <https://github.com/Mobsya/aseba-target-thymio2/blob/master/behavior.c#L182>`_,
    see ``static void behavior_leds_buttons(void)``.

    The original behavior ticks at 50Hz.
    It should be assigned to a Thymio, like in

    >>> thymio.control_step_callback = ThymioLEDButtonsBehavior()
    """

    DT: float = 0.02

    def __init__(self) -> None:
        self.button_counter = np.zeros(5)

    def __call__(self, thymio: PhysicalObject, dt: SupportsFloat) -> None:
        """Executes the behavior"""
        assert isinstance(thymio, Thymio2)
        self.tick(Thymio2AsebaAdapter(thymio), float(dt))

    def tick(self, thymio: Thymio2AsebaAdapter, dt: float) -> None:
        steps = dt / self.DT
        values = np.asarray(thymio.thymio.buttons)
        self.button_counter[values] = np.minimum(
            32, self.button_counter[values > 0] + 3 * steps)
        self.button_counter[values == 0] = 0
        bc = self.button_counter.astype(int)
        if bc[0]:
            leds = bc[[0, 0, 0, 0]]
        else:
            leds = bc[[1, 2, 3, 4]]
        thymio.call_leds_buttons(*leds)
