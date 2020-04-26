API
====

.. currentmodule:: pyenki


Vector
--------

A type alias for Tuple[float, float] as (x, y).

Polygon
--------

A type alias for Iterable[:ref:`Vector`], where points should be ordered counter-clockwise.


Part
--------

A type alias for Tuple[:ref:`Polygon`, float] representing a convex right prism as (base, height).


Color
---------------

.. autoclass:: Color


PhysicalObject
---------------

.. autoclass:: PhysicalObject


RectangularObject
-----------------

.. autoclass:: RectangularObject


CircularObject
---------------

.. autoclass:: CircularObject

ConvexObject
---------------

.. autoclass:: ConvexObject

CompositeObject
---------------

.. autoclass:: CompositeObject


IRCommEvent
---------------

.. autoclass:: IRCommEvent

DifferentialWheeled
---------------

.. autoclass:: DifferentialWheeled
   :members: reset_encoders


E-puck
-------

.. autoclass:: Epuck
  :members: controlStep, set_led_ring


Marxbot
-------

.. autoclass:: Marxbot
  :members: controlStep


Thymio2
---------------

.. autoclass:: Thymio2
   :members: controlStep, set_led_top, set_led_bottom_left, set_led_bottom_right, set_led_buttons, set_led_prox, set_led_circle, set_led_left_red, set_led_right_red, set_led_left_blue, set_led_right_blue

World
------

.. autoclass:: World
  :members:


WorldView
----------

.. autoclass:: WorldView
  :members:
