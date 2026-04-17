from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING, SupportsFloat

if TYPE_CHECKING:
    from thymio_behaviors import ThymioAsebaProtocol
    Behavior = Callable[[ThymioAsebaProtocol, float], None]

from ... import Controller, PhysicalObject, Thymio2
from .thymio import Thymio2AsebaAdapter


def make_controller_from_thymio_behavior(thymio: PhysicalObject,
                                         behavior: Behavior) -> Controller:
    assert isinstance(thymio, Thymio2)
    aseba = Thymio2AsebaAdapter(thymio)

    def control(obj: PhysicalObject, dt: SupportsFloat) -> None:
        """Executes the behavior"""
        assert obj is thymio

        aseba.update()
        behavior(aseba, float(dt))
        aseba.actuate()

    return control
