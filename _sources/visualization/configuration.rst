=============
Configuration
=============

When import the pyenki viewer with ::

   import pyenki.viewer

it checks whether pyenki has been built with Qt support (native renderer) and/or PySide6 is installed (Python renderer). 
In case no implementation is available, it raises an exception.
In case one implementation is available, it selects it.
In case both implementations are available, it selects the one specified by the ``PYENKI_NATIVE_RENDER`` environment variable. Set it to ``1``, ``ON`` or ``TRUE`` to select the native renderer, **before** the viewer is imported. That is, set it to ::

   PYENKI_NATIVE_RENDER=0

to select the Python renderer and to ::

   PYENKI_NATIVE_RENDER=1

to select the native renderer.


We can check which implementation has been selected with ::

   >>> pyenki.viewer.use_native
   False




