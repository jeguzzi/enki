=====
Types
=====



Aliases
=======

.. currentmodule:: pyenki

.. py:type:: Vector
   :canonical: typing.Annotated[numpy.typing.NDArray[numpy.float64], '[2, 1]']

   A two dimensional vector in cm.

.. py:type:: VectorLike
   :canonical: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, '[2, 1]']

   Anything that can be converted to a :py:type:`Vector`.

.. py:type:: Array1D
   :canonical:  typing.Annotated[numpy.typing.NDArray[numpy.float64], '[n]']

   A one-dimensional array of floats

.. py:type:: IntArray1D
   :canonical:  typing.Annotated[numpy.typing.NDArray[numpy.int64], '[n]']

   A one-dimensional array of integers.

.. py:type:: Array2D
   :canonical:  typing.Annotated[numpy.typing.NDArray[numpy.float64], '[n]']

   A two-dimensional array of floats

.. py:type:: Image
   :canonical:  typing.Annotated[numpy.typing.NDArray[numpy.uint8], '[n, m, 3]']

   An RGB image with 8 bits per color.

.. py:type:: ARGBImage
   :canonical:  typing.Annotated[numpy.typing.NDArray[numpy.uint8], '[n, m, 4]']

   An ARGB image with 8 bits per color.

.. py:type:: ARGBImageLike
   :canonical:  typing.Annotated[numpy.typing.ArrayLike, numpy.uint8, '[n, m, 4]']

   Anything that can be converted to a :py:type:`ARGBImage`.

.. py:type:: Controller
   :canonical: collections.abc.Callable[[pyenki.PhysicalObject, SupportsFloat], None]

   The type of callbacks that can be assigned to :py:attr:`pyenki.PhysicalObject.control_step_callback` ::

      def callback(PhysicalObject: obj, time_step: float) -> None: ...

   The first argument is the object to control, the second the time step (in seconds).

.. currentmodule:: pyenki.viewer

.. py:type:: Vector3
   :canonical: typing.Annotated[numpy.typing.NDArray[numpy.float64], '[3, 1]']

   A three dimensional vector in cm.

.. py:type:: Vector3Like
   :canonical: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, '[3, 1]']

   Anything that can be converted to a :py:type:`Vector3`.




Color
=====

.. currentmodule:: pyenki

.. autoclass:: Color
   :members:
   :exclude-members: __new__, __init__
