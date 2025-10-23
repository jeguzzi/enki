====
API
====

.. currentmodule:: pyenki


Type aliases
============

.. py:type:: Vector

   :canonical: numpy.ndarray[tuple[typing.Literal[2]], numpy.dtype[numpy.float64]]

   A two dimensional vector in cm.

.. py:type:: Polygon

   :canonical: Sequence[:py:type:`Vector`]

   A sequence of vertices ordered counter-clockwise.

.. py:type:: Part

   :canonical: tuple[:py:type:`Polygon`, float] | tuple[:py:type:`Polygon`, float, Sequence[:py:class:`Color`]]

   An object (prismatic) part defined as ``(base, height)`` or ``(base, height, face_colors)`` where ``face_colors`` and ``base`` must have the same length.

   .. warning::
       At the moment textures (``face_colors``) are used to compute the sensors (   cameras) response but are ignored when displaying the object.

Color
=====

.. autoclass:: Color


PhysicalObject
==============

.. autoclass:: PhysicalObject
   :members:

.. autofunction:: RectangularObject

.. autofunction:: CircularObject

.. autofunction:: CompositeObject

   .. warning::
       At the moment textures (``face_colors``) are used to compute the sensors (   cameras) response but are ignored when displaying the object.

.. autofunction:: ConvexObject

   .. warning::
       At the moment textures (``face_colors``) are used to compute the sensors (   cameras) response but are ignored when displaying the object.

Robots
======

DifferentialWheeled
-------------------

.. autoclass:: DifferentialWheeled
   :members:


E-puck
-------

.. autoclass:: EPuck
  :members:


Marxbot
-------

.. autoclass:: Marxbot
  :members:


Thymio2
-------

.. autoclass:: IRCommEvent

.. autoclass:: Thymio2
   :members:



World
=====

.. autoclass:: World
  :members:


.. WorldView
.. ----------

.. .. autoclass:: WorldView
..   :members:
