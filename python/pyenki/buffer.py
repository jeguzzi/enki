from __future__ import annotations

import asyncio
import time
from collections.abc import Collection
from typing import Any

import jupyter_rfb  # type: ignore[import-untyped]
import numpy
import numpy.typing

import pyenki


class EnkiRemoteFrameBuffer(jupyter_rfb.RemoteFrameBuffer  # type: ignore[misc]
                            ):
    """
    Renders a world in a jupyter notebook by calling :py:meth:`pyenki.World.render`.

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

    def __init__(self,
                 world: pyenki.World | None = None,
                 camera_position: pyenki.Vector = numpy.zeros(2),
                 camera_altitude: float = 30,
                 camera_yaw: float = 0,
                 camera_pitch: float = -numpy.pi / 2,
                 camera_is_ortho: bool = False):
        """
        Constructs a new instance.

        :param      world:            The world
        :param      camera_position:  The camera position
        :param      camera_altitude:  The camera altitude
        :param      camera_yaw:       The camera yaw
        :param      camera_pitch:     The camera pitch
        :param      camera_is_ortho:  Whether the camera uses an orthographic projection
        """
        super().__init__(resizable=True)
        self._camera_position: numpy.typing.NDArray[
            numpy.float64] = numpy.array([*camera_position, camera_altitude],
                                         dtype=numpy.float64)
        self.camera_yaw = camera_yaw
        self.camera_pitch = camera_pitch
        self.camera_is_ortho = camera_is_ortho
        self.world = world
        self._p: tuple[float, float] | None = None

    @property
    def camera_position(self) -> pyenki.Vector:
        return self._camera_position[:2]

    @camera_position.setter
    def camera_position(self, value: pyenki.Vector) -> None:
        self._camera_position[:2] = value

    @property
    def camera_altitude(self) -> float:
        return float(self._camera_position[2])

    @camera_altitude.setter
    def camera_altitude(self, value: float) -> None:
        self._camera_position[2] = value

    async def run_async(
        self,
        time_step: float,
        duration: float,
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
        for _ in range(int(duration / time_step)):
            self.world.step(time_step)
            self.request_draw()
            for r in synch:
                r.request_draw()
            await asyncio.sleep(time_step / factor)

    def run(
        self,
        time_step: float,
        duration: float,
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
        for _ in range(int(duration / time_step)):
            self.world.step(time_step)
            self.request_draw_sync()
            for r in synch:
                r.request_draw_sync()
            time.sleep(time_step / factor)

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
        if not self._rfb_draw_requested:  # type: ignore[has-type]
            self._rfb_draw_requested = True
            self._rfb_cancel_lossless_draw()
            self._rfb_maybe_draw()

    def move_camera(self,
                    target_position: pyenki.Vector,
                    target_altitude: float = 0,
                    target_distance: float = 100,
                    camera_yaw: float | None = None,
                    camera_pitch: float | None = None) -> None:
        """
        Move the camera so to point towards the target.
        Same interface as :py:meth:`pyenki.WorldView.move_camera`.

        :param      target_position:  The target horizontal position in cm.
        :param      target_altitude:  The target vertical position in cm.
        :param      target_distance:  The distance to the target.
        :param      camera_yaw:       Optionally sets the camera yaw.
        :param      camera_pitch:     Optionally sets the camera pitch.
        """
        if camera_yaw is not None:
            self.camera_yaw = camera_yaw
        if self.camera_is_ortho:
            self._camera_position = numpy.array(
                [*target_position, target_altitude + target_distance],
                dtype=numpy.float64)
        else:
            if camera_pitch is not None:
                self.camera_pitch = camera_pitch
            e = numpy.array(
                (numpy.cos(self.camera_yaw) * numpy.cos(self.camera_pitch),
                 numpy.sin(self.camera_yaw) * numpy.cos(self.camera_pitch),
                 numpy.sin(self.camera_pitch)))
            p = numpy.array([*target_position, target_altitude],
                            dtype=numpy.float64)
            self._camera_position = p - target_distance * e
        self.request_draw()

    def point_camera(self,
                     target_position: pyenki.Vector,
                     target_altitude: float = 0,
                     position: pyenki.Vector | None = None,
                     altitude: float | None = None) -> None:
        """
        Rotate the camera so to point towards the target.
        Same interface as :py:meth:`pyenki.WorldView.point_camera`.

        :param      target_position:  The target horizontal position in cm.
        :param      target_altitude:  The target vertical position in cm.
        :param      position:       Optionally sets the camera position.
        :param      altitude:     Optionally sets the camera altitude.
        """
        if not self.camera_is_ortho:
            if position is not None:
                self.camera_position = position
            if altitude is not None:
                self.camera_altitude = altitude
            dp = numpy.array([*target_position, target_altitude],
                             dtype=numpy.float64) - self._camera_position
            self.camera_yaw = numpy.arctan2(dp[1], dp[0])
            self.camera_pitch = numpy.arctan2(dp[2], numpy.linalg.norm(dp[:2]))
            self.request_draw()

    def handle_event(self, event: dict[str, Any]) -> None:
        event_type = event.get("event_type", None)
        if event_type == "resize":
            self.size = event["width"], event["height"], event["pixel_ratio"]
        elif event_type == "pointer_down" and event["button"] == 1:
            self._p = event["x"], event["y"]
            self.request_draw()
        elif event_type == "pointer_up":
            self._p = None
            self.request_draw()
        elif event_type == "wheel":
            delta = event["dy"] / self.size[1] * 6
            e = numpy.array(
                (numpy.cos(self.camera_yaw) * numpy.cos(self.camera_pitch),
                 numpy.sin(self.camera_yaw) * numpy.cos(self.camera_pitch),
                 numpy.sin(self.camera_pitch))) * delta
            self._camera_position -= e
            self.request_draw()
        elif event_type == "pointer_move" and self._p is not None:
            dx = event["x"] - self._p[0]
            dy = event["y"] - self._p[1]
            self.camera_yaw += dx / self.size[0] * 3
            self.camera_pitch += dy / self.size[1] * 3
            self._p = (event["x"], event["y"])
            self.request_draw()

    def get_frame(self) -> numpy.typing.NDArray[numpy.uint8]:
        assert self.world
        image = self.world.render(camera_position=self.camera_position,
                                  camera_altitude=self.camera_altitude,
                                  camera_yaw=self.camera_yaw,
                                  camera_pitch=self.camera_pitch,
                                  camera_is_ortho=self.camera_is_ortho,
                                  width=self.size[0],
                                  height=self.size[1])
        return image
