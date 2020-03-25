# find Python
find_package (Python COMPONENTS Interpreter Development)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
set(CMAKE_MACOSX_RPATH 1)
endif()

# make sure we have the right modules
if (Python_FOUND AND Qt5Widgets_FOUND AND Qt5OpenGL_FOUND)
	message(STATUS "Python libs and executable found, looking for boost::python")
	FIND_PACKAGE(Boost COMPONENTS system python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR})
	if (Boost_FOUND)
		message(STATUS "boost::python found, generating python bindings")
		include_directories(${PROJECT_SOURCE_DIR} ${Python_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
		Python_add_library(pyenki MODULE enki.cpp)
		target_link_libraries(pyenki  PRIVATE enki enkiviewer ${Boost_LIBRARIES} Qt5::OpenGL Qt5::Widgets ${OPENGL_gl_LIBRARY})
		install(TARGETS pyenki LIBRARY DESTINATION "${Python_SITELIB}")
	else (Boost_FOUND)
		message(WARNING "You need boost::python to generate Python bindings")
	endif (Boost_FOUND)
else ()
	message(WARNING "Python libs or executable, or Qt5, or OpenGL not found, skipping Python bindings")
endif ()
