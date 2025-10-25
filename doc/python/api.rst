====
API
====

.. currentmodule:: pyenki.pyenki


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
   :members:
   :exclude-members: __new__, __init__


PhysicalObject
==============

.. autoclass:: PhysicalObject
   :members:
   :exclude-members: __new__, __init__

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

Robot
-----

.. autoclass:: Robot
   :exclude-members: __new__, __init__

DifferentialWheeled
-------------------

.. autoclass:: DifferentialWheeled
   :members:
   :exclude-members: __new__, __init__


E-puck
-------

.. autoclass:: EPuck
   :members:
   :exclude-members: __new__


Marxbot
-------

.. autoclass:: Marxbot
   :members:
   :exclude-members: __new__


Thymio2
-------

.. autoclass:: IRCommEvent
   :exclude-members: __new__, __init__

.. autoclass:: Thymio2
   :members:
   :exclude-members: __new__

World
=====

.. autoclass:: World
   :members:
   :exclude-members: __new__


WorldView
=========

.. autoclass:: WorldView
   :members:
   :exclude-members: __new__


.. currentmodule:: pyenki

Helpers
=======

Remote buffer
-------------

.. automodule:: pyenki.buffer
   :members:


Video
-----

.. automodule:: pyenki.video
   :members:

