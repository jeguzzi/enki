# Changelog in fork https://github.com/jeguzzi/enki

## Branch qt6

Adds support for Qt6. 

- Cmake minimal version is now 3.16
- Replaced `QGLWidget` with `QOpenGLWidget`. `QGLWidget` has been deprecated since Qt5.4 and has been removed in Qt6. 
	- replaced native (`GLuint`) with `QOpenGLTexture` textures.
- Switched to non-const `ViewerUserData::draw`
- For Thymios, individual (LED) textures are now stored in a `std::map`
- Switched to shared OpenGL context. Textures are handled differently in Qt5 and Qt6. Added `EnkiApplication` to initialize/deinitialize the context automatically. 
	- Models now shares most textures (which they load just once per application)
	- Physical objects holds copies of models instead of reference to the same object.
	- Deprecated `ViewerWidget ::managedObjectsAliases` and removed `ViewerWidget ::managedObjects`. Adding models requires now to override `makeUserData`.
 	- Models do not depends on viewer anymore.
- Added world setter to `ViewerWidget`
	- `ViewerWidget` supports now null worlds.
- Made updating the world optional in the timer callback. 
- Added `loadTexture` that flips texture images using non-deprecated API.
- Added examples that show the updated viewer at play.

### Required downstream changes

Dependencies should

- use `Enki::EnkiApplication` instead of `QtApplication`
- when creating a new custom robot model:
	- Constructor takes no argument
	- add `static void init()` and fill with OpenGL texture and OpenGL lists initialization.
	- add `static void deinit` and fill with OpenGL texture and OpenGL lists deinitialization.
	- in case add static members `static inline std::vector<GLuint> lists` and 
	  `static inline std::vector<std::unique_ptr<QOpenGLTexture>> textures`
	- replace `glBindTexture(GL_TEXTURE_2D, textures[i]);` with `textures[i]->bind();`
- remove `managedObjects`
- remove third (size, integer) argument of `renderText`

## Branch pybind11

Switched from python-boost to pybind11. The Python module keeps almost the same interface.

- added pyproject.toml
- wheels can now be build using `python -m build -w`
- `Enki::Vector` is now exposed as a numpy array, which it now requires now.

## Branch proximity_sensors

Cleaner implementation of prox and prox-comm based on `ircomm` branch.

- added IRComm
- added IRSensorRealistic
- exposed number of rays and aperture in IRSensor constructor
- added a different range and search_range in IRSensor
- added virtual methods to specialize initialization and finalization of global interactions

## Branch reproducible

All random generators belongs now to the world and are seeded for reproducibility.

- replaced Random.h with RandomWIthSeed.h
- added a `Random` instance to the world.
- added seed argument to `World` constructor

## Branch complete_pyenki

In Python:
- Changed names to comply with PEP8
- Added methods to set all Thymios LEDs
- Added callbacks (control and collision)
- Added objects accessors
- Added physical objects factory methods
- Added aseba-compatible versions of the Thymio methods/accessors
- Switched sensor readings to return numpy arrays
- Added e-Puck scanner
- Added e-Puck capabilities
- Added marxbot
- Added viewer and GUI functions
- Added methods to render a world
- Added numpy random generator to world
- Added docstrings
- Added examples
- Added docs
- Added package with helpers to generate videos and display live views of the world in a notebook.

In C++:
- Exposed `ViewerWidget ` timer period, and if/how it should update the world.
- Added real-time factor to `ViewerWidget`
- Added flag to not display helper widgets to `ViewerWidget`
- Added orthographic projection to `ViewerWidget`
- Added setter for motor noise.
- Added default robot names.
- Corrected distance from cylinders calculation in Circular camera.
- Renamed ``EnkiApplication::init` to `EnkiApplication::setup`
- Added `EnkiApplication::init`, `EnkiApplication::run`, `EnkiApplication::cleanup`.

## Branch pyviewer

It re-implements the world viewer class (and renderers) in Python using PySide6. OpenGL (v4) shaders and VAOs replace the C++ implementation based on OpenGL fixed pipeline. Shaders replaces dynamic textures to implement Thymio LEDs.

The main advantages are:
- more modern OpenGL (4 vs 1.X)
- simplified Python wheels because rendering does not requires building/linking against Qt
- can use rendering with any version of PySide6 (vs only the one linked against the same version of Qt6).

### Added

- added subpackage `pyenki.viewer` with Pyside6-based OpenGL renderer used by an offscreen renderer and an QOpenGLWidget subclass. `pyenki.viewer.__init__` selects whether to load the native or the python version depending on availability and the `PYENKI_NATIVE_VIEWER` environment variable.
- exposed Thymio LEDs colors with `pyenki.Thymio2.led_colors`
- exposed mouse/touch events (for physical objects)
- exposed Thymio buttons
- exposed world ground texture
- exposed world dimensions
- exposed world wall types
- exposed PhysicalObject::Part as `pyenki.PhysicalObject.Part`
- moved PhysicalObject factory constructors to init
- added `share` argument to `pyenki.viewer.init`
- added buffer protocol to `pyenki.Color`
- exposed additional colors.
- added `Camera` and `HasCamera`
- added "proper" offscreen rendering to the native viewer
- added methods to query the 3D position of a pixel in the world.
- added interaction to `pyenki.buffer.EnkiRemoteFramebuffer` (select, move, track)

### Removed

- `pyenki.CircularObject`, `pyenki.RectangularObject`, `pyenki.CompositeObject`, `pyenki.ConvexObject`.

### Changed

- split pybind11 wrapper in two modules: the second requires Qt, exposes the native viewer, and is loaded in `pyenki.viewer.native`.
- moved `pyenki.WorldView` to `pyenki.viewer.WorldView`
- moved `pyenki.init_ui`, `pyenki.run_ui`, `pyenki.cleanup_ui` to `pyenki.viewer.init`, `pyenki.viewer.run`, `pyenki.viewer.cleanup`.
- refactored docs.




