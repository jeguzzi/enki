Examples
========

Hello Thymio
------------

In the most common case, you want to subclass one or more robots adding the appropriate controller.
Then, you setup up the world, adding as many objects as needed.
Finally, you may either run the simulation in real time inside a Qt application,
or write your own loop where to call ``World.step``.


In this example, a Thymio will advance as long as there is not a obstacle (a wall) in front of it,
when it will stop and switch the LED color to red from green.


  .. include:: ../../example/hello_thymio.py
    :code: Python


Proximity Communication
-----------------------

In this example, two Thymios placed in front of each other, transmit messages using proximity communication.
After each control step, we print the received messages.


.. include:: ../../example/two_thymios_comm.py
  :code: Python


Interactive GUI
---------------

The QWidget that display the world can be run either two modes:
- embedded in a standalone QtApplication, like in :ref:`Hello Thymio`, that blocks until it terminates
- or using an already running QtApplication, which does not block and allow to visualize the world
while manipulating it in an interactive session (e.g., in a jupyter notebook or an IPython console)

For instance, this script

.. include:: ../../example/interactive_view.py
  :code: Python

will spawn a live world view when run inside an IPython console

.. code-block:: Bash

  $ ipython --gui qt5
  >>> %run interactive_view
