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

	

