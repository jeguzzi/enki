============
Installation
============

We support Python 3.10-3.14. Enki is written in C++: you need CMake and a C++ compiler supporting C++-17.


No visualization
================

To install the package without visualization capabilities, run

.. code::

   pip install git+https://github.com/jeguzzi/enki --config-settings=cmake.define.WITH_QT=

Visualization
=============

The package comes with two implementations (one in C++, one in Python) of almost identical visualizations based on OpenGL and Qt. It can be configured to build the C++ implementation and link it against Qt5 or Qt6 shared libraries. Alternatively, it can be built without a C++ visualization and use instead a Python implementation that uses PySide6. The two options can coexist provided that pyenki and PySide6 are linked against the **same** Qt distribution, else it will fail at runtime, complaining about multiple versions for the same symbols (from the different versions of Qt shared libraries).

We refer to :doc:`visualization/introduction` for a description of the differences between the two implementations. All things being equal, we recommend using the Python implementation as it uses a more modern version of OpenGL, does not require building against Qt, and integrates with the official Qt bindings (PySide6).

Python-based
------------

To enable visualization using the Python implementation of the renderer, run

.. code::

   pip install "pyenki[viewer] @ git+https://github.com/jeguzzi/enki" --config-settings=cmake.define.WITH_QT=


Native, C++-based
-----------------

To enable visualization using the C++-implementation of the renderer, you need to install either Qt5 or Qt6, and select the version.

.. tabs::
   
   .. tab:: Qt5
   
      .. code::

         pip install git+https://github.com/jeguzzi/enki --config-settings=cmake.define.WITH_QT=5

   .. tab:: Qt6
   
      .. code::

         pip install git+https://github.com/jeguzzi/enki --config-settings=cmake.define.WITH_QT=6

Extras
======

Add the extra to be able to visualize the simulation in a notebook and to make videos from simulations, for example:

.. code::

   pip install "pyenki[viewer,all] @ git+https://github.com/jeguzzi/enki" --config-settings=cmake.define.WITH_QT=

