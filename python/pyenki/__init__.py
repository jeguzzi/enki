from __future__ import annotations

from typing import Annotated, TypeAlias

import numpy
import numpy.typing

from .pyenki import (Color, DifferentialWheeled, EPuck, Marxbot,
                     PhysicalObject, Robot, Thymio2, World)

Vector: TypeAlias = Annotated[numpy.typing.NDArray[numpy.float64], '[2, 1]']
VectorLike: TypeAlias = Annotated[numpy.typing.ArrayLike, numpy.float64,
                                  '[2, 1]']
Array1D: TypeAlias = Annotated[numpy.typing.NDArray[numpy.float64], '[n]']
Array2D: TypeAlias = Annotated[numpy.typing.NDArray[numpy.float64], '[n, m]']
IntArray1D: TypeAlias = Annotated[numpy.typing.NDArray[numpy.int64], '[n, m]']
Image: TypeAlias = Annotated[numpy.typing.NDArray[numpy.uint8], '[n, m, 3]']
ARGBImage: TypeAlias = Annotated[numpy.typing.NDArray[numpy.uint8],
                                 '[n, m, 4]']
ARGBImageLike: TypeAlias = Annotated[numpy.typing.ArrayLike, numpy.uint8,
                                     '[n, m, 4]']

__all__ = [
    'Color', 'DifferentialWheeled', 'EPuck', 'Marxbot', 'PhysicalObject',
    'Robot', 'Thymio2', 'World'
]
