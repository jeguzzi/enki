from __future__ import annotations

from typing import Any, TYPE_CHECKING

import moviepy as mpy  # type: ignore[import-untyped]

import pyenki
from pyenki.viewer import render

if TYPE_CHECKING:
    import numpy.typing


def make_video(world: pyenki.World,
               time_step: float,
               duration: float,
               factor: float = 1.0,
               **kwargs: Any) -> mpy.VideoClip:
    """
    Generate a video by simulating the world for a while.

    :param      world:      The world to be simulated
    :param      time_step:  The time step of the simulation
    :param      duration:   The duration of the simulation
    :param      factor:     The real-time factor. If larger than one, it will speed up the video.
    :param      kwargs:     The keywords arguments passed to :py:func:`pyenki.viewer.render`

    :returns:   The video clip.
    """
    time = 0.0

    def make_frame(t: float) -> numpy.typing.NDArray[numpy.uint8]:
        nonlocal time
        t = t * factor
        while time + time_step < t:
            world.step(time_step)
            time += time_step
        dt = t - time
        if dt > 0:
            world.step(dt)
            time += dt

        return render(world, **kwargs)

    return mpy.VideoClip(make_frame, duration=duration / factor)
