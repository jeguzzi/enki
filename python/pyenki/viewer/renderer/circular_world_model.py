from __future__ import annotations

import numpy as np
from numpy.typing import NDArray
from OpenGL import GL  # type: ignore[import-untyped]
from PySide6.QtGui import QMatrix4x4, QOpenGLContext
from PySide6.QtOpenGL import (QOpenGLBuffer, QOpenGLShaderProgram,
                              QOpenGLTexture, QOpenGLVertexArrayObject)

from ... import World
from .utils import (circle, create_vao_with_vertices, enable_texture,
                    forward_transform, functions, switch_context)


def circular_world_data(n: int = 60) -> NDArray[np.float32]:
    bottom = circle(n=n)
    delta = np.array([0, 0, 1], dtype=np.float32)
    top = bottom + delta
    rtop = top
    rtop_far = (circle(n=n) * 5 + delta)
    center = np.zeros(3)
    side = np.stack([bottom, top], axis=1).reshape(-1, 3)
    top = np.stack([rtop, rtop_far], axis=1).reshape(-1, 3)
    vertices = np.concatenate([[center], bottom, side, top]).astype(np.float32)
    side = -circle(n=n)
    side = np.stack([side, side], axis=1).reshape(-1, 3)
    e = np.array([0, 0, 1], dtype=np.float32)
    normals = np.concatenate([[e] * (n + 1), side,
                              [e] * (2 * n)]).astype(np.float32)
    # textures = np.concatenate([
    #     np.zeros((n + 1, 2), dtype=np.float32),
    #     np.array([[[0, 0], [0.5, 0.5]] * n], dtype=np.float32).reshape(-1, 2),
    #     np.zeros((2 * n, 2), dtype=np.float32)
    # ], axis=0)
    textures_center = np.array([[0.5, 0.5]], dtype=np.float32)
    textures_bottom = (bottom[:, :2] + 1) / 2
    textures_side = np.array([[[x, 0], [x, 1]] for x in np.linspace(0, 1, n)],
                             dtype=np.float32).reshape(-1, 2)
    textures_top = np.zeros((2 * n, 2), dtype=np.float32)
    textures = np.concatenate(
        [textures_center, textures_bottom, textures_side, textures_top])

    data = np.concatenate([vertices, textures, normals], axis=1).reshape(-1, 8)
    return data  # type: ignore


class CircularWorldModel:

    def __init__(self) -> None:
        self.N = 60
        self.textures: dict[str, QOpenGLTexture] = {}
        self.vbo: QOpenGLBuffer | None = None
        self.vao: QOpenGLVertexArrayObject | None = None

    def destroy(self) -> None:
        for texture in self.textures.values():
            texture.destroy()
        if self.vbo:
            self.vbo.destroy()
        self.vbo = None
        if self.vao:
            self.vao.destroy()
        self.vao = None

    def init(self, ctx: QOpenGLContext) -> None:
        with switch_context(ctx):
            self.vao, self.vbo = create_vao_with_vertices(
                circular_world_data(self.N))

    def draw(self, world: World, wall_height: float,
             program: QOpenGLShaderProgram, camera: QMatrix4x4,
             projection: QMatrix4x4, ctx: QOpenGLContext) -> None:
        if not self.vao:
            self.init(ctx)
        assert self.vao
        f = functions()
        t = QMatrix4x4()
        t.scale(world.radius, world.radius, wall_height)
        forward_transform(camera * t, program)
        with QOpenGLVertexArrayObject.Binder(self.vao):
            f.glDrawArrays(GL.GL_TRIANGLE_FAN, 0, self.N + 1)
            enable_texture(False, program)
            f.glDrawArrays(GL.GL_TRIANGLE_STRIP, 3 * self.N + 1, 2 * self.N)
            f.glDrawArrays(GL.GL_TRIANGLE_STRIP, self.N + 1, 3 * self.N + 1)
