Installation
=============

This package provides Python bindings to the enki simulator
using `Boost::Python <https://www.boost.org/doc/libs/1_72_0/libs/python/doc/html>`_.

This is a fork of https://github.com/enki-community/enki with the changes listed in the :ref:`Changelog`.
We thanks the authors of enki very much for their fantastic work.

Requirements
------------

You need to have Python and Boost::Python installed.
In case you want to visualize the simulation, you need to install Qt too.
For macOS, using homebrew, run:

.. code-block:: bash

  brew install python3 boost-python3 qt5


Compiling from source
---------------------

Clone and compile the code:

.. code-block:: bash

    git clone https://github.com/jeguzzi/enki.git --branch ir_comm
    cd enki
    cmake .
    make
    make install
