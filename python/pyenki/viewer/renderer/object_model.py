from __future__ import annotations

from collections import defaultdict


from ... import PhysicalObject, World
from .cylinder_model import CylinderModel
from .object_part_model import ObjectPartModel
from .utils import forward_color, forward_transform, get_transform
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from PySide6.QtOpenGL import QOpenGLShaderProgram
    from PySide6.QtGui import QMatrix4x4, QOpenGLContext


class ObjectModel:

    def __init__(self) -> None:
        self.parts: dict[int, ObjectPartModel] = {}
        self.part_worlds: dict[int, set[World]] = defaultdict(set)
        self.cylinder = CylinderModel()

    def destroy(self) -> None:
        self.cylinder.destroy()
        for part in self.parts.values():
            part.destroy()
        self.parts.clear()
        self.part_worlds.clear()

    def remove_world(self, world: World) -> None:
        keys = []
        for key, worlds in self.part_worlds.items():
            worlds.remove(world)
            if not worlds:
                keys.append(key)
        for key in keys:
            del self.part_worlds[key]
            if key in self.parts:
                self.parts[key].destroy()
                del self.parts[key]
                print('removed part model')

    def add_world(self, world: World) -> None:
        for obj in world.static_objects:
            for part in obj.parts:
                self.part_worlds[hash(part)].add(world)

    def draw(self, obj: PhysicalObject, program: QOpenGLShaderProgram,
             camera: QMatrix4x4, projection: QMatrix4x4,
             ctx: QOpenGLContext) -> None:
        forward_color(obj.color, program)
        if obj.is_cylindric:
            self.cylinder.draw(obj, program, camera, projection, ctx)
        else:
            t = camera * get_transform(obj)
            forward_transform(t, program)
            for part in obj.parts:
                if hasattr(part, "_model_key"):
                    key = part._model_key
                else:
                    key = hash(part)
                    part._model_key = key  # type: ignore[attr-defined]
                if key not in self.parts:
                    self.parts[key] = ObjectPartModel(part)
                self.parts[key].render(program, ctx)
