=========
Renderers
=========

pyenki comes with two implementation of a Qt-based world renderer.

The first wraps the original implementation written in C++ and using (now deprecated) fixed-function OpenGL pipeline. The second, is written in Python using the official Qt6 bindings (PySide6) and uses a more modern OpenGL pipeline compatible with OpenGL version 4. 

They are almost functionally equivalent. 
Both supports visualizing a world in a interactive GUI in a Qt application or in a Jupyter notebook, rendering it as an image, and generating a video from simulations.

The following table list their differences:

.. list-table:: Frozen Delights!
   :widths: 20 20 20
   :header-rows: 1

   * - Functionality
     - Native renderer
     - Python renderer
   * - Shadows
     - implemented
     - not *yet* implemented.
   * - Physical object textures
     - not implemented
     - implemented
   * - Requires Qt at build-time
     - yes, either Qt5 or Qt6
     - no
   * - Requires Qt Python libraries
     - no
     - yes, PySide6

The Python-based renderer does not require Qt to install pyenki, resulting in no external dependency apart from a C++ compiler.






