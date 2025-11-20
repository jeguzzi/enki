from __future__ import annotations

from PySide6.QtGui import QMatrix4x4

from ... import Marxbot
from .robot_model import RobotModel
from .utils import (enable_texture, forward_color, forward_transform, get_transform,
                    get_wheel_angles)
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from PySide6.QtOpenGL import QOpenGLShaderProgram
    from PySide6.QtGui import QOpenGLContext


class MarxbotModel(RobotModel[Marxbot]):

    PARTS = {'base': 'marxbot_base', 'wheel': 'marxbot_wheel'}
    TEXTURES = {'main': 'marxbot.png'}

    def draw(self, robot: Marxbot, program: QOpenGLShaderProgram,
             camera: QMatrix4x4, projection: QMatrix4x4,
             ctx: QOpenGLContext) -> None:
        if not self.textures:
            self.init(ctx)
        assert self.textures
        enable_texture(True, program)
        self.textures['main'].bind()
        t = camera * get_transform(robot)
        forward_color(robot.color, program)
        forward_transform(t, program)
        self.parts['base'].render(ctx)
        wheel_radius = 2.9
        left, right = get_wheel_angles(robot, wheel_radius)
        r = QMatrix4x4()
        r.translate(0, 0, wheel_radius)
        r.rotate(left, 0, 1, 0)
        forward_transform(t * r, program)
        self.parts['wheel'].render(ctx)
        r = QMatrix4x4()
        r.translate(0, 0, wheel_radius)
        r.rotate(right, 0, 1, 0)
        r.rotate(180, 0, 0, 1)
        forward_transform(t * r, program)
        self.parts['wheel'].render(ctx)
