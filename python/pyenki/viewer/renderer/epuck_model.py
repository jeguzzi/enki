from __future__ import annotations

from PySide6.QtGui import QMatrix4x4, QOpenGLContext
from PySide6.QtOpenGL import QOpenGLShaderProgram

from ... import Color, EPuck
from .robot_model import RobotModel
from .utils import (enable_texture, forward_color, forward_transform, get_transform,
                    get_wheel_angles, to_vector)


class EPuckModel(RobotModel[EPuck]):

    PARTS = {
        'body': 'epuck_body',
        'rest': 'epuck_rest',
        'ring': 'epuck_ring',
        'left_wheel': 'epuck_wheel_left',
        'right_wheel': 'epuck_wheel_right'
    }
    TEXTURES = {'main': 'epuck.png'}

    def draw(self, robot: EPuck, program: QOpenGLShaderProgram,
             camera: QMatrix4x4, projection: QMatrix4x4,
             ctx: QOpenGLContext) -> None:
        if not self.textures:
            self.init(ctx)
        assert self.textures
        enable_texture(True, program)
        self.textures['main'].bind()
        wheel_radius = 2.1
        t = QMatrix4x4()
        t.translate(0, 0, wheel_radius)
        t = camera * t * get_transform(robot)
        forward_color(Color.white, program)
        forward_transform(t, program)
        self.parts['body'].render(ctx)
        self.parts['rest'].render(ctx)
        program.setUniformValue("material.emission", to_vector(robot.color))  # type: ignore[call-overload]
        self.parts['ring'].render(ctx)
        program.setUniformValue("material.emission", to_vector(Color.black))  # type: ignore[call-overload]
        left, right = get_wheel_angles(robot, wheel_radius)
        r = QMatrix4x4()
        r.rotate(left, 0, 1, 0)
        forward_transform(t * r, program)
        self.parts['left_wheel'].render(ctx)
        r = QMatrix4x4()
        r.rotate(right, 0, 1, 0)
        forward_transform(t * r, program)
        self.parts['right_wheel'].render(ctx)
