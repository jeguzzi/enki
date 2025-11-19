from collections.abc import Callable

import numpy as np

from .. import PhysicalObject, World
from .camera import Camera, Vector3
from .utils import get_object_at

Pixel = tuple[int, int]
PixelPositionGetter = Callable[[Pixel], Vector3 | None]


class UI:

    def __init__(self, camera: Camera) -> None:
        self.last_pixel: Pixel | None = None
        self.selected_object: PhysicalObject | None = None
        self.camera = camera

    def on_mouse_press(self, pixel: Pixel, world: World | None,
                       pixel_position: PixelPositionGetter) -> bool:
        self.last_pixel = pixel
        if not self.selected_object and world:
            p = pixel_position(pixel)
            if p is not None:
                self.selected_object = get_object_at(world,
                                                     p[:2],
                                                     tolerance=0.2)
                return True
        return False

    def on_mouse_release(self) -> bool:
        self.selected_object = None
        self.last_pixel = None
        return True

    def on_mouse_move(self, pixel: Pixel,
                      get_position_of_pixel: PixelPositionGetter,
                      left_button: bool, right_button: bool,
                      shift: bool) -> bool:
        if not self.last_pixel:
            return False
        dx = pixel[0] - self.last_pixel[0]
        dy = pixel[1] - self.last_pixel[1]
        self.last_pixel = pixel
        if self.selected_object:
            if right_button:
                sensitivity = 10 / (1 + self.width)
                self.selected_object.angle -= sensitivity * dx
                return True
            p = get_position_of_pixel(pixel)
            if left_button and p is not None:
                self.selected_object.position = p[:2]
                self.selected_object.velocity = (0, 0)
                self.selected_object.angular_speed = 0
                return True

        if shift:
            sensitivity = -(1 + 0.1 * self.camera.position[2]) * 0.1
            self.camera.position += sensitivity * dy * self.camera.forward
        elif left_button:
            sensibility = 20.0 + 2. * self.camera.position[2]
            size_factor = 1.0 + (self.width + self.height) / 2
            self.camera.position -= sensibility * (
                dx * self.camera.left + dy * self.camera.up) / size_factor
        else:
            sensitivity = 4.0
            self.camera.yaw -= sensitivity * dx / (1 + self.width)
            delta = 0.01
            self.camera.pitch = np.clip(
                self.camera.pitch - sensitivity * dy / (1 + self.height),
                -np.pi / 2 + delta, np.pi / 2 - delta)
        return True

    def on_wheel(self, delta: float) -> bool:
        sensitivity = (1 + 0.1 * self.camera.position[2]) * 0.003
        self.camera.position += sensitivity * delta * self.camera.forward
        self.camera.position[2] = max(0, self.camera.position[2])
        return True

    def on_resize(self, width: int, height: int) -> bool:
        self.camera.set_viewport(width, height)
        self.width = width
        self.height = height
        return True
