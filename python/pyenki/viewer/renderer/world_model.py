from __future__ import annotations

from PySide6.QtOpenGL import QOpenGLTexture

from ... import World
from .circular_world_model import CircularWorldModel
from .flat_world_model import FlatWorldModel
from .square_world_model import SquareWorldModel
from .utils import enable_texture, forward_color, from_numpy_image
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from PySide6.QtOpenGL import QOpenGLShaderProgram
    from PySide6.QtGui import QMatrix4x4, QOpenGLContext


class WorldModel:

    def __init__(self) -> None:
        self.circular = CircularWorldModel()
        self.square = SquareWorldModel()
        self.flat = FlatWorldModel()
        self.ground_textures: dict[int, QOpenGLTexture] = {}
        self.textures: dict[str, QOpenGLTexture] = {}

    def destroy(self) -> None:
        self.circular.destroy()
        self.square.destroy()
        self.flat.destroy()
        for texture in self.ground_textures.values():
            texture.destroy()
        self.ground_textures.clear()
        for texture in self.textures.values():
            texture.destroy()
        self.textures.clear()

    # def init(self, ctx: QOpenGLContext) -> None:
    #     with switch_context(ctx):
    #         self.textures = {
    #             'world':
    #             QOpenGLTexture(QImage(":/textures/world.png").flipped()),
    #             'wall':
    #             QOpenGLTexture(QImage(":/textures/wall.png").flipped())
    #         }

    def get_ground_texture(self, world: World) -> QOpenGLTexture:
        if id(world) not in self.ground_textures:
            texture = QOpenGLTexture(from_numpy_image(world.ground_texture))
            texture.setMinMagFilters(QOpenGLTexture.Filter.Nearest, QOpenGLTexture.Filter.Nearest)
            self.ground_textures[id(world)] = texture
        return self.ground_textures[id(world)]

    def draw(self, world: World, wall_height: float,
             program: QOpenGLShaderProgram, camera: QMatrix4x4,
             projection: QMatrix4x4, ctx: QOpenGLContext) -> None:
        forward_color(world.walls_color, program)
        if world.has_ground_texture:
            self.get_ground_texture(world).bind()
        enable_texture(world.has_ground_texture, program)
        if world.walls_type == World.WallsType.CIRCULAR:
            self.circular.draw(world, wall_height, program, camera, projection,
                               ctx)
        elif world.walls_type == World.WallsType.SQUARE:
            self.square.draw(world, wall_height, program, camera, projection,
                             ctx)
        else:
            self.flat.draw(world, program, camera, projection, ctx)
