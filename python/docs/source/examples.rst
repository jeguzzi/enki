Example
=======

In the most common case, you want to subclass one or more robots adding the appropriate controller.
Then, you setup up the world, adding as many objects as needed.
Finally, you may either run the simulation in real time inside a Qt application,
or write your own loop where to call ``World.step``.


In this example, a Thymio will advance as long as there is not a obstacle (a wall) in front of it,
when it will stop and switch the LED color to red from green.


  .. include:: ../../example/hello_thymio.py
    :code: Python
