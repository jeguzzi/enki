cmake_minimum_required(VERSION 3.16)

project(Enki)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# additional CMake modules
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)
option(BUILD_EXAMPLES "Whether to build examples" ON)
option(BUILD_TESTS "Whether to build tests" ON)
option(BUILD_PYTHON "Whether to build the python module" ON)


enable_testing()

set(WITH_QT "6" CACHE STRING "Which Qt version (5 or 6) to use. Leave empty not to use Qt.")
if(WITH_QT MATCHES "6")
set(QT_VERSION_REQ "6")
find_package(Qt6 COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets)
elseif(WITH_QT MATCHES "5")
set(QT_VERSION_REQ "5")
find_package(Qt5 COMPONENTS Core Gui Widgets OpenGL)
endif()
if (Qt6_FOUND)
	message("Enki will be built with Qt6 support")
elseif(Qt5_FOUND)
	message("Enki will be built with Qt5 support")
else()
	message("Enki will be built without Qt support")
endif()

# library version
set(LIB_INSTALL_DIR lib CACHE FILEPATH "Where to install libraries")
set(LIB_VERSION_MAJOR 2) # Must be bumped for incompatible ABI changes
set(LIB_VERSION_MINOR 0)
set(LIB_VERSION_PATCH 0)
set(LIB_VERSION_STRING ${LIB_VERSION_MAJOR}.${LIB_VERSION_MINOR}.${LIB_VERSION_PATCH})

add_subdirectory(enki)

if (Qt6_FOUND OR Qt5_FOUND)
	find_package(OpenGL REQUIRED)
	find_package(SDL2)
	add_subdirectory(viewer)
endif()

if (BUILD_PYTHON)
add_subdirectory(python)
endif()
if (BUILD_TESTS)
add_subdirectory(tests)
endif()
if (BUILD_EXAMPLES)
add_subdirectory(examples)
endif()

# Documentation
find_package(Doxygen)
option(BUILD_DOCUMENTATION
	"Create and install the HTML based API documentation (requires Doxygen)"
	${DOXYGEN_FOUND})

if (BUILD_DOCUMENTATION)
	if (NOT DOXYGEN_FOUND)
		message(FATAL_ERROR "Doxygen is needed to build the documentation.")
	endif()

	set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in)
	set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

	configure_file(${doxyfile_in} ${doxyfile} @ONLY)

	add_custom_target(doc
		COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Generating API documentation with Doxygen"
		VERBATIM)

	install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION share/doc OPTIONAL)
endif()

# generate configure file
configure_file(enkiConfig.cmake.in "${PROJECT_BINARY_DIR}/enkiConfig.cmake" @ONLY)
if (WIN32 AND NOT CYGWIN)
	set(INSTALL_CMAKE_DIR CMake)
else()
	if (APPLE)
		set(INSTALL_CMAKE_DIR Frameworks/enki.framework/Resources/CMake/)
	else()
		set(INSTALL_CMAKE_DIR share/enki/CMake)
	endif()
endif()
install(FILES "${PROJECT_BINARY_DIR}/enkiConfig.cmake" DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)
