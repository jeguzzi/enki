import numpy as np

from .. import PhysicalObject
from .camera import HasCamera
from .types import Pixel
from .utils import get_object_at


class UI:

    def __init__(self, widget: HasCamera) -> None:
        self.last_pixel: Pixel | None = None
        self._selected_object: PhysicalObject | None = None
        self.is_moving_object: bool = False
        self.widget = widget
        self.camera = widget.camera

    @property
    def selected_object(self) -> PhysicalObject | None:
        return self._selected_object

    @selected_object.setter
    def selected_object(self, value: PhysicalObject | None) -> None:
        if value is not self._selected_object:
            self.widget.tracked_object = None
            if not self.is_moving_object:
                self._selected_object = value

    def on_mouse_press(self, pixel: Pixel, left_button: bool) -> bool:
        self.last_pixel = pixel
        if not left_button:
            return False
        if self.widget.world:
            p = self.widget.get_position_of_pixel(pixel)
            if p is not None:
                self.selected_object = get_object_at(self.widget.world,
                                                     p[:2],
                                                     tolerance=0.2)
                return True
        return False

    def on_mouse_release(self) -> bool:
        self.is_moving_object = False
        self.last_pixel = None
        return True

    def on_mouse_double_click(self) -> bool:
        if self.selected_object:
            self.widget.tracked_object = self.selected_object
        return True

    def on_mouse_move(self, pixel: Pixel, left_button: bool,
                      right_button: bool, shift: bool) -> bool:
        if not self.last_pixel:
            return False
        dx = pixel[0] - self.last_pixel[0]
        dy = pixel[1] - self.last_pixel[1]
        self.last_pixel = pixel
        if self.selected_object and not self.widget.tracked_object:
            self.is_moving_object = True
            if right_button:
                sensitivity = 10 / (1 + self.width)
                self.selected_object.angle -= sensitivity * dx
                return True
            p = self.widget.get_position_of_pixel(pixel)
            if left_button and p is not None:
                self.selected_object.position = p[:2]
                self.selected_object.velocity = (0, 0)
                self.selected_object.angular_speed = 0
                return True
            return False

        if left_button and not self.widget.tracked_object:
            if shift:
                sensitivity = -(1 + 0.1 * self.camera.position[2]) * 0.1
                self.camera.position += sensitivity * dy * self.camera.forward
            else:
                sensibility = 20.0 + 2. * self.camera.position[2]
                size_factor = 1.0 + (self.width + self.height) / 2
                self.camera.position -= sensibility * (
                    dx * self.camera.left + dy * self.camera.up) / size_factor
            return True
        if right_button:
            sensitivity = 4.0
            delta = sensitivity * dx / (1 + self.width)
            if self.widget.tracked_object:
                self.widget.tracking_angle -= delta
            else:
                self.camera.yaw -= delta
            epsilon = 0.01
            self.camera.pitch = np.clip(
                self.camera.pitch - sensitivity * dy / (1 + self.height),
                -np.pi / 2 + epsilon, np.pi / 2 - epsilon)
            return True
        return False

    def on_wheel(self, delta: float) -> bool:
        if self.widget.tracked_object:
            self.widget.tracking_distance = max(
                1.0, self.widget.tracking_distance * (1 - 0.0003 * delta))
        else:
            sensitivity = (1 + 0.1 * self.camera.position[2]) * 0.003
            self.camera.position += sensitivity * delta * self.camera.forward
            self.camera.position[2] = max(0, self.camera.position[2])
        return True

    def on_resize(self, width: int, height: int) -> bool:
        self.camera.set_viewport(width, height)
        self.width = width
        self.height = height
        return True
