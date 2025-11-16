from __future__ import annotations

from OpenGL import GL  # type: ignore[import-untyped]
from PySide6.QtGui import QMatrix4x4, QOpenGLContext
from PySide6.QtOpenGL import QOpenGLShaderProgram

from ... import Thymio2
from .robot_model import RobotModel
from .utils import (enable_texture, forward_color, forward_transform, get_transform,
                    get_wheel_angles, load_program, setup_program,
                    switch_context, functions)


class Thymio2Model(RobotModel[Thymio2]):

    PARTS = {'body': 'thymio2_body', 'wheel': 'thymio2_wheel'}
    TEXTURES = {
        'body': 'thymio-body-texture.png',
        'led0': 'thymio-body-diffusionMap0.png',
        'led1': 'thymio-body-diffusionMap1.png',
        'led2': 'thymio-body-diffusionMap2.png',
        'wheel': 'thymio-wheel-texture.png'
    }

    def __init__(self) -> None:
        super().__init__()
        self.body_program: QOpenGLShaderProgram | None = None

    def destroy(self) -> None:
        super().destroy()
        if self.body_program:
            del self.body_program
        self.body_program = None

    def init(self, ctx: QOpenGLContext) -> None:
        super().init(ctx)
        with switch_context(ctx):
            f = functions()
            self.body_program = load_program(vs='default', fs='thymio_body')
            self.body_program.bind()
            texture = self.body_program.uniformLocation("u_texture")
            led0 = self.body_program.uniformLocation("led0")
            led1 = self.body_program.uniformLocation("led1")
            led2 = self.body_program.uniformLocation("led2")
            self.leds_location = self.body_program.uniformLocation(
                "led_colors")
            f.glUniform1i(texture, 0)
            f.glUniform1i(led0, 1)
            f.glUniform1i(led1, 2)
            f.glUniform1i(led2, 3)
            self.body_program.release()

    def draw(self, robot: Thymio2, program: QOpenGLShaderProgram,
             camera: QMatrix4x4, projection: QMatrix4x4,
             ctx: QOpenGLContext) -> None:
        if not self.textures:
            self.init(ctx)
        assert self.textures

        f = functions()
        program.release()
        assert self.body_program
        self.body_program.bind()
        setup_program(projection=projection, program=self.body_program)
        f.glActiveTexture(GL.GL_TEXTURE0 + 0)
        self.textures['body'].bind()
        f.glActiveTexture(GL.GL_TEXTURE0 + 1)
        self.textures['led0'].bind()
        f.glActiveTexture(GL.GL_TEXTURE0 + 2)
        self.textures['led1'].bind()
        f.glActiveTexture(GL.GL_TEXTURE0 + 3)
        self.textures['led2'].bind()
        f.glActiveTexture(GL.GL_TEXTURE0 + 0)
        data = robot.led_colors.flatten().tolist()
        f.glUniform4fv(self.leds_location, len(data), data)
        t_body = QMatrix4x4()
        t_body.translate(2.5, 0, 0)
        t = camera * get_transform(robot)
        t_body = t * t_body
        forward_color(robot.color, self.body_program)
        forward_transform(t_body, self.body_program)
        self.parts['body'].render(ctx)
        self.body_program.release()
        program.bind()
        enable_texture(True, program)
        wheel_radius = 2.1
        left, right = get_wheel_angles(robot, wheel_radius)
        self.textures['wheel'].bind()
        r = QMatrix4x4()
        r.translate(0, 4, wheel_radius)
        r.rotate(left, 0, 1, 0)
        m = t * r
        forward_color(robot.color, program)
        forward_transform(m, program)
        self.parts['wheel'].render(ctx)
        r = QMatrix4x4()
        r.translate(0, -4, wheel_radius)
        r.rotate(right, 0, 1, 0)
        r.rotate(180, 0, 0, 1)
        m = t * r
        forward_transform(m, program)
        self.parts['wheel'].render(ctx)
