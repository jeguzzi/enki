from __future__ import annotations

import importlib.util
import os
import sys
import warnings
from collections.abc import Callable
from typing import SupportsFloat, SupportsInt

if sys.version_info >= (3, 11):
    from typing import Unpack
else:
    from typing_extensions import Unpack

from .. import Image, PhysicalObject, World
from .camera import Camera, rotate, to_3d
from .types import (CameraConfig, InitProtocol, PositionOfPixelGetterProtocol,
                    RenderProtocol, RunInViewerProtocol, RunProtocol,
                    SaveImageProtocol, Vector3, Vector3Like, WorldViewProtocol)

use_native: bool | None = None
"""
Whether is using the native (C++) or the Python implementation of the viewer.
"""
WorldView: type[WorldViewProtocol]
init: InitProtocol
run: RunProtocol
render: RenderProtocol
save_image: SaveImageProtocol
run_in_viewer: RunInViewerProtocol
cleanup: Callable[[], None]
get_position_of_pixel: PositionOfPixelGetterProtocol


def _has_native_viewer() -> bool:
    return importlib.util.find_spec("pyenki.pyenki_viewer") is not None


def _has_pyside() -> bool:
    return importlib.util.find_spec("PySide6") is not None


def _get_env_native_viewer() -> bool:
    p = _has_pyside()
    n = _has_native_viewer()
    if not p and not n:
        raise ImportError(
            "Rendering requires PySide6 or pyenki built with Qt support")
    has_value = 'PYENKI_NATIVE_VIEWER' in os.environ
    value = os.environ.get('PYENKI_NATIVE_VIEWER',
                           '0').upper() not in ('0', 'NO', "OFF")
    if not p and not value:
        if has_value:
            warnings.warn("No PySide6 installed, switch to native viewer",
                          stacklevel=2)
        return True
    if not n and value:
        if has_value:
            warnings.warn(
                "Pyenki without native viewer, switch to Python viewer",
                stacklevel=2)
        return False
    return value


def _setup_viewer(value: bool) -> None:
    global WorldView
    global init
    global run
    global cleanup
    global save_image
    global run_in_viewer
    global render
    global get_position_of_pixel
    global use_native
    use_native = value
    if value:
        from . import native

        def save_image(world: World,
                       path: str,
                       /,
                       walls_height: SupportsFloat = 10,
                       width: SupportsInt = 640,
                       height: SupportsInt = 360,
                       selected_object: PhysicalObject | None = None,
                       **config: Unpack[CameraConfig]) -> None:
            native.save_image(world,
                              path,
                              walls_height=walls_height,
                              width=width,
                              height=height,
                              selected_object=selected_object,
                              **config)

        def render(world: World,
                   /,
                   walls_height: SupportsFloat = 10,
                   width: SupportsInt = 640,
                   height: SupportsInt = 360,
                   selected_object: PhysicalObject | None = None,
                   **config: Unpack[CameraConfig]) -> Image:
            return native.render(world,
                                 walls_height=walls_height,
                                 width=width,
                                 height=height,
                                 selected_object=selected_object,
                                 **config)

        def run_in_viewer(world: World,
                          /,
                          fps: SupportsFloat = 30,
                          time_step: SupportsFloat = 0,
                          factor: SupportsFloat = 1,
                          helpers: bool = True,
                          walls_height: SupportsFloat = 10,
                          duration: SupportsFloat = -1,
                          **config: Unpack[CameraConfig]) -> None:
            native.run_in_viewer(world,
                                 fps=fps,
                                 time_step=time_step,
                                 factor=factor,
                                 helpers=helpers,
                                 walls_height=walls_height,
                                 duration=duration,
                                 **config)

        WorldView = native.WorldView  # type: ignore[assignment]
        init = native.init
        run = native.run
        cleanup = native.cleanup
        get_position_of_pixel = native.get_position_of_pixel
    else:
        from . import offscreen_renderer, utils, widget

        WorldView = widget.WorldView
        init = utils.init
        run = utils.run
        cleanup = utils.cleanup
        save_image = offscreen_renderer.save_image
        run_in_viewer = widget.run_in_viewer
        render = offscreen_renderer.render
        get_position_of_pixel = offscreen_renderer.get_position_of_pixel

    World.save_image = save_image  # type: ignore[attr-defined]
    World.render = render  # type: ignore[attr-defined]
    World.run_in_viewer = run_in_viewer  # type: ignore[attr-defined]


_setup_viewer(_get_env_native_viewer())

__all__ = [
    'init', 'run', 'cleanup', 'render', 'save_image', 'WorldView', 'Camera',
    'use_native', 'get_position_of_pixel', 'run_in_viewer', 'to_3d', 'rotate',
    'Vector3', 'Vector3Like', 'CameraConfig'
]
