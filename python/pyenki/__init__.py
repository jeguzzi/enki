from typing import Literal, TypeAlias

import numpy

from .pyenki import *  # noqa

Vector: TypeAlias = numpy.ndarray[  # type: ignore
    tuple[Literal[2]], numpy.dtype[numpy.float64]]
