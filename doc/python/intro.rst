============
Introduction
============

This package provides Python bindings to the **enki** simulator
using `Pybind11 <https://pybind11.readthedocs.io/en/stable/>`_. This is a fork of the `original repository <https://github.com/enki-community/enki>`_. We thank the original authors of enki very much for their fantastic work.

With **pyenki** users can create worlds populated with static objects and three types of robots (E-puck, Marxbot, and Thymio). They can define controllers for the robots: read sensors, move wheels, and change LEDs. They can run batch simulation or real-time simulation visualized in interactive views.

.. figure:: world.png
   :width: 75%

   The three robot types implemented in pyenki.



Enki is 2.5-dimensional: collisions and physics are computed based only on 2-dimensional geometry, while sensing is affected by the vertical dimension too. It is very computationally efficient and has no external requirements, except for the optional visualization.


Differences with the original version
=====================================

We have modernized a bit the code, in particular adding support for Qt6 (the original supported Qt5 only which is now EOL) and switching from Boost-Python to Pybind11. We have also implemented an alternative renderer in Python, which makes the Qt dependency optional at build-time.
We also changed names to comply with PEP8 and added docstrings.

The other main changes are:

- random sampling is now completely determined by a random seed, allowing to reproduce simulations;
- implemented Thymio buttons and IR communication;
- exposed all robots and their sensors and actuators to Python;
- added callbacks as an alternative to sub-class robots;
- added off-screen rendering.


:ref:`Changelog` lists the complete changes.


