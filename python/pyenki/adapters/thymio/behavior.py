from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from thymio_behaviors import ThymioAsebaProtocol
    Behavior = Callable[[ThymioAsebaProtocol, float], None]

from ... import Controller, PhysicalObject, Thymio2
from .thymio import Thymio2AsebaAdapter


def make_controller_from_thymio_behavior(thymio: PhysicalObject,
                                         behavior: Behavior) -> Controller:
    assert isinstance(thymio, Thymio2)
    aseba = Thymio2AsebaAdapter(thymio)
    return aseba.make_controller(behavior)
