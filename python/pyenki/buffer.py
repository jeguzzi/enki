from __future__ import annotations

import asyncio
import sys
import time
import typing
from collections.abc import Collection
from typing import Any

if sys.version_info >= (3, 11):
    from typing import Unpack
else:
    from typing_extensions import Unpack

import jupyter_rfb  # type: ignore[import-untyped]

import pyenki
from pyenki.viewer import get_position_of_pixel, render
from pyenki.viewer.camera import HasCamera
from pyenki.viewer.types import CameraConfig, Pixel, Vector3
from pyenki.viewer.ui import UI

if typing.TYPE_CHECKING:
    import numpy as np
    import numpy.typing


class EnkiRemoteFrameBuffer(
        jupyter_rfb.RemoteFrameBuffer,  # type: ignore[misc]
        HasCamera):
    """
    Renders a world in a jupyter notebook by calling :py:func:`pyenki.viewer.render`.

    Attributes:
        world (pyenki.World | None): The world to display
        camera_position (pyenki.Vector):  The camera position
        camera_altitude (float): the vertical position of the camera.
        camera_yaw (float): the camera rotation around the vertical axis.
        camera_pitch (float): the camera vertical rotation.
        camera_is_ortho (bool): whether the camera uses an orthographic projection.

    Example::

        >>> import pyenki
        >>> world = pyenki.World()
        >>> from pyenki.buffer import EnkiRemoteFrameBuffer
        >>> view = EnkiRemoteFrameBuffer(world=world)
        >>> view.move_camera(target_position=(0, 0), target_altitude=5,
                             camera_yaw=1, camera_pitch=-0.5, target_distance=40)
        >>> view
    """

    _rfb_draw_requested: bool

    def __init__(self,
                 world: pyenki.World | None = None,
                 **camera_config: Unpack[CameraConfig]):
        """
        Constructs a new instance.

        :param      world:            The world
        :param      camera_position:  The camera position
        :param      camera_altitude:  The camera altitude
        :param      camera_yaw:       The camera yaw
        :param      camera_pitch:     The camera pitch
        :param      camera_is_ortho:  Whether the camera uses an orthographic projection
        """
        # super().__init__(resizable=True)
        jupyter_rfb.RemoteFrameBuffer.__init__(self, resizable=True)
        HasCamera.__init__(self, world=world, **camera_config)
        self.world = world
        self._ui = UI(self)
        self.size: tuple[int, int, int] = (0, 0, 1)

    async def run_async(
        self,
        time_step: float,
        duration: float = -1,
        factor: float = 1,
        synch: Collection[EnkiRemoteFrameBuffer] = tuple()
    ) -> None:
        """
        Runs a simulation and updates the view for a while.

        :param      time_step:  The time step of the simulation
        :param      duration:   The duration of the simulation
        :param      factor:     The real-time factor. If larger than one, the simulation
                                will run faster then real-time.
        :param      synch:      Which other buffers should be redrawn.
        """
        if not self.world:
            return
        t = 0.0
        while duration <= 0 or t < duration:
            self.world.step(time_step)
            self.request_draw()
            for r in synch:
                r.request_draw()
            await asyncio.sleep(time_step / factor)
            t += time_step

    def run(
        self,
        time_step: float,
        duration: float = -1,
        factor: float = 1,
        synch: Collection[EnkiRemoteFrameBuffer] = tuple()
    ) -> None:
        """
        Runs a simulation and updates the view for a while.

        :param      time_step:  The time step of the simulation
        :param      duration:   The duration of the simulation
        :param      factor:     The real-time factor. If larger than one, the simulation
                                will run faster then real-time.
        :param      synch:      Which other buffers should be redrawn.
        """
        if not self.world:
            return
        t = 0.0
        while duration <= 0 or t < duration:
            self.world.step(time_step)
            self.request_draw_sync()
            for r in synch:
                r.request_draw_sync()
            time.sleep(time_step / factor)
            t += time_step

    async def tick_async(self, fps: float) -> None:
        """
        Updates the view and sleeps for a while.
        """
        self.request_draw()
        await asyncio.sleep(1 / fps)

    def tick(self, fps: float) -> None:
        """
        Updates the view and sleeps for a while.
        """
        self.request_draw_sync()
        time.sleep(1 / fps)

    def request_draw_sync(self) -> None:
        """
        Similar to :py:meth:`jupyter_rfb.RemoteFrameBuffer.request_draw`
        but works outside of an even loop.
        """
        if not self._rfb_draw_requested:
            self._rfb_draw_requested = True
            self._rfb_cancel_lossless_draw()
            self._rfb_maybe_draw()

    def width(self) -> int:
        return self.size[0]

    def height(self) -> int:
        return self.size[1]

    def get_pixel(self, event: dict[str, Any]) -> Pixel:
        return int(event["x"]), int(event["y"])

    def get_position_of_pixel(self, pixel: Pixel) -> Vector3 | None:
        if self.world:
            x = pixel[0] / self.size[0]
            y = pixel[1] / self.size[1]
            return get_position_of_pixel((x, y))
        return None

    @staticmethod
    def get_buttons(event: dict[str, Any]) -> tuple[bool, bool]:
        control = 'Control' in event['modifiers']
        button_1 = 1 in event['buttons'] or event['button'] == 1
        button_2 = 2 in event['buttons'] or event['button'] == 2
        left_button = button_1 and not control
        right_button = (button_1 and control) or button_2
        return left_button, right_button

    def handle_event(self, event: dict[str, Any]) -> None:
        event_type = event.get("event_type")
        if event_type == "close":
            # print('closing')
            pass
        if event_type == "double_click":
            if self._ui.on_mouse_double_click():
                self.request_draw()
        if event_type == "resize":
            self.size = int(event["width"]), int(event["height"]), int(
                event["pixel_ratio"])
            w, h, _ = self.size
            self._ui.on_resize(int(w), int(h))
        elif event_type == "pointer_down":
            if self._ui.on_mouse_press(self.get_pixel(event),
                                       *self.get_buttons(event)):
                self.request_draw()
        elif event_type == "pointer_up":
            if self._ui.on_mouse_release(*self.get_buttons(event)):
                self.request_draw()
        elif event_type == "wheel":
            delta = -event["dy"] * 5
            if self._ui.on_wheel(delta):
                self.request_draw()
        elif event_type == "pointer_move":
            if self._ui.on_mouse_move(self.get_pixel(event),
                                      *self.get_buttons(event),
                                      shift='Shift' in event['modifiers']):
                self.request_draw()

    def get_frame(self) -> numpy.typing.NDArray[np.uint8]:
        assert self.world
        self.update_camera()
        image = render(self.world,
                       width=self.size[0] * self.size[2],
                       height=self.size[1] * self.size[2],
                       selected_object=self._ui.selected_object,
                       **self.camera_config)
        self._last_image = image
        return image
