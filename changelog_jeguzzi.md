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


