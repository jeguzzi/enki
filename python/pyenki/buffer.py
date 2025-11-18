from __future__ import annotations

import asyncio
import time
import typing
from collections.abc import Collection
from typing import Any

import jupyter_rfb  # type: ignore[import-untyped]
import numpy as np
import numpy.typing

import pyenki
from pyenki.viewer import render
from pyenki.viewer.camera import CameraConfig, HasCamera
from pyenki.viewer.offscreen_renderer import get_position_of_pixel
from pyenki.viewer.utils import get_object_at


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
                 **camera_config: typing.Unpack[CameraConfig]):
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
        self._p: tuple[float, float] | None = None
        self.selected_object: pyenki.PhysicalObject | None = None
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

    def handle_event(self, event: dict[str, Any]) -> None:
        event_type = event.get("event_type", None)
        if event_type == "close":
            print('closing')
        if event_type == "resize":
            self.size = int(event["width"]), int(event["height"]), int(event["pixel_ratio"])
            w, h, _ = self.size
            self.camera.set_viewport(int(w), int(h))
        elif event_type == "pointer_down" and event["button"] == 1:
            self._p = event["x"], event["y"]
            if self.world:
                x = int(event["x"]) * self.size[2]
                y = int(event["y"]) * self.size[2]
                p = get_position_of_pixel((x, y))
                if p is not None:
                    print(p)
                    obj = get_object_at(self.world, p[:2], tolerance=0.2)
                    if obj:
                        self.selected_object = obj
                        self.request_draw()
        elif event_type == "pointer_up":
            self._p = None
            self.selected_object = None
            self.request_draw()
        elif event_type == "wheel":
            delta = event["dy"] / self.size[1] * 6
            e = self.camera.forward * delta
            self.camera.position -= e
            self.request_draw()
        elif event_type == "pointer_move" and self._p is not None:
            dx = event["x"] - self._p[0]
            dy = event["y"] - self._p[1]
            self._p = (event["x"], event["y"])
            if self.selected_object:
                if 'Control' in event['modifiers']:
                    sensitivity = 10 / (1 + self.size[0])
                    self.selected_object.angle -= sensitivity * dx
                else:
                    x = int(event["x"]) * self.size[2]
                    y = int(event["y"]) * self.size[2]
                    p = get_position_of_pixel((x, y))
                    if p is not None:
                        self.selected_object.position = p[:2]
                        self.selected_object.velocity = (0, 0)
                        self.selected_object.angular_speed = 0
            else:
                if 'Shift' in event['modifiers']:
                    sensitivity = -(1 + 0.1 * self.camera_altitude) * 0.1
                    self.camera.position += sensitivity * dy * self.camera.forward
                elif 'Control' not in event['modifiers']:
                    sensibility = 20.0 + 2. * self.camera_altitude
                    size_factor = 1.0 + (self.size[0] + self.size[1]) / 2
                    self.camera.position -= sensibility * (
                        dx * self.camera.left +
                        dy * self.camera.up) / size_factor
                else:
                    sensitivity = 4.0
                    self.camera_yaw -= sensitivity * dx / (1 + self.size[0])
                    delta = 0.01
                    self.camera_pitch = np.clip(
                        self.camera_pitch - sensitivity * dy /
                        (1 + self.size[1]), -np.pi / 2 + delta,
                        np.pi / 2 - delta)

                    # self.camera_yaw += dx / self.size[0] * 3
                    # self.camera_pitch += dy / self.size[1] * 3

            self.request_draw()

    def get_frame(self) -> numpy.typing.NDArray[np.uint8]:
        assert self.world

        image = render(self.world,
                       width=self.size[0] * self.size[2],
                       height=self.size[1] * self.size[2],
                       selected_object=self.selected_object,
                       **self.camera_config)
        self._last_image = image
        return image
