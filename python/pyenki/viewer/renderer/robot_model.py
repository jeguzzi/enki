from __future__ import annotations

import abc
from typing import Generic, TypeVar

import numpy as np
from OpenGL import GL  # type: ignore[import-untyped]
from PySide6.QtGui import QImage, QMatrix4x4, QOpenGLContext
from PySide6.QtOpenGL import (QOpenGLBuffer, QOpenGLShaderProgram,
                              QOpenGLTexture, QOpenGLVertexArrayObject)
from shiboken6 import VoidPtr

from ... import Robot
from .utils import (create_vao_with_indices_and_vertices, functions,
                    load_numpy_array, switch_context)

from pathlib import Path

RobotType = TypeVar("RobotType", bound=Robot)


class RobotModelPart:

    def __init__(self, prefix: str) -> None:
        self.prefix = prefix
        self.ibo: QOpenGLBuffer | None = None
        self.vbo: QOpenGLBuffer | None = None
        self.vao: QOpenGLVertexArrayObject | None = None
        self.number_of_indices = 0

    def load_data(self) -> tuple[np.ndarray, np.ndarray]:
        root = Path(__file__).parent / "objects"
        indices = load_numpy_array(root / f'{self.prefix}_i.npy')
        vertices = load_numpy_array(root / f'{self.prefix}_v.npy')
        return indices, vertices

    def init(self, ctx: QOpenGLContext) -> None:
        with switch_context(ctx):
            indices, vertices = self.load_data()
            self.vao, self.ibo, self.vbo = create_vao_with_indices_and_vertices(
                indices, vertices)
            # print(indices.shape, vertices.shape)
            self.number_of_indices = indices.size

    def render(self, ctx: QOpenGLContext) -> None:
        if self.vao is None:
            self.init(ctx)
        assert self.vao is not None
        f = functions()
        with QOpenGLVertexArrayObject.Binder(self.vao):
            f.glDrawElements(
                GL.GL_TRIANGLES,
                self.number_of_indices,
                GL.GL_UNSIGNED_SHORT,
                VoidPtr(0)  # type: ignore
            )

    def destroy(self) -> None:
        if self.ibo:
            self.ibo.destroy()
        self.ibo = None
        if self.vbo:
            self.vbo.destroy()
        self.vbo = None
        if self.vao:
            self.vao.destroy()
        self.vao = None


class RobotModel(abc.ABC, Generic[RobotType]):
    TEXTURES: dict[str, str] = {}
    PARTS: dict[str, str] = {}

    def __init__(self) -> None:
        self.parts = {
            name: RobotModelPart(prefix)
            for name, prefix in self.PARTS.items()
        }
        self.textures: dict[str, QOpenGLTexture] = {}

    def destroy(self) -> None:
        for texture in self.textures.values():
            texture.destroy()
        self.textures.clear()
        for part in self.parts.values():
            part.destroy()
        self.parts.clear()

    def init(self, ctx: QOpenGLContext) -> None:
        root = Path(__file__).parent.parent / "textures"
        with switch_context(ctx):
            self.textures = {
                name: QOpenGLTexture(QImage(str(root / file)).flipped())
                for name, file in self.TEXTURES.items()
            }

    @abc.abstractmethod
    def draw(self, robot: RobotType, program: QOpenGLShaderProgram,
             camera: QMatrix4x4, projection: QMatrix4x4,
             ctx: QOpenGLContext) -> None:
        ...
