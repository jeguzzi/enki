==========
Simulation
==========

In the most common case, you add appropriate controllers for one or more robots, either by sub-classing or by assigning callbacks.  
Then, you set up the world, adding as many objects as needed.
Finally, you may run and visualize the simulation in real time, or in batch-mode with :py:meth:`pyenki.World.run` or by calling :py:meth:`pyenki.World.step` in a loop.

Minimal Thymio
--------------

In this example, two Thymio robots exchange IR messages,
which we log at each time step. This is the Python equivalent to the `C++ example <https://github.com/jeguzzi/enki/blob/rolling/examples/minimal_thymio/enkiMinimalThymio.cpp>`_

.. include:: ../../../examples/python/minimal_thymio.py
   :code: Python

Hello Thymio
------------

In this example, a Thymio will advance as long as there is not an obstacle (a wall) in front of it, when it will stop and switch the LED color to red from green.

.. include:: ../../../examples/python/hello_thymio.py
   :code: Python

Launching it with ``--gui`` will visualize a real-time simulation.

.. figure:: images/hello.png
   :width: 75%

Thymio Buttons
--------------

In this example, a Thymio will react to buttons. Try to press any button to change its color.

.. include:: ../../../examples/python/thymio_buttons.py
   :code: Python

.. figure:: images/buttons.png
   :width: 75%