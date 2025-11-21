=============
Visualization
=============

Viewer
======

The Qt widget that displays the world can be run in a script, using a blocking loop, like in :ref:`Hello Thymio`, or in an interactive session that does not block and allows to visualize the world while manipulating it.

Standalone
----------

In this example, we instantiate a :py:class:`pyenki.viewer.WorldView` that shows a world running in real-time:

.. include:: ../../../examples/python/world_view.py
  :code: Python

Integrated in PySide or PyQt 
----------------------------

The :py:class:`pyenki.viewer.WorldView` can be integrated as any other Qt widget in a more complex interface.  

In these examples, we create a window with the world visualized from two different points of view:

.. tabs:: 

   .. tab:: PyQt6

      .. include:: ../../../examples/python/world_view_qt.py
         :code: Python

   .. tab:: PySide6
   
      .. include:: ../../../examples/python/world_view_pyside.py
         :code: Python

In a notebook
=============

We support visualizing a simulation inside Jupyter notebooks thanks to `jupyter_rfb <https://jupyter-rfb.readthedocs.io>`_:

.. code-cell::

   import pyenki

   world = pyenki.World()
   thymio = pyenki.Thymio2()
   thymio.left_wheel_target_speed = 10
   world.add_object(thymio)

.. code-cell::

   from pyenki.buffer import EnkiRemoteFrameBuffer
   
   w = EnkiRemoteFrameBuffer(world=world)
   w

.. code-cell::

   await w.run_async(time_step=0.1, duration=5)

.. seealso::

   `Hello World.ipynb <https://github.com/jeguzzi/enki/blob/rolling/examples/python/Hello World.ipynb>`_ and `RemoteBuffer.ipynb <https://github.com/jeguzzi/enki/blob/rolling/examples/python/RemoteBuffer.ipynb>`_

Render an image
===============

To render a single image (and/or save it), we can run:

.. include:: ../../../examples/python/render.py
  :code: Python

Generate a video
================

To generate a video from a simulation, we can run:

.. include:: ../../../examples/python/video.py
  :code: Python
