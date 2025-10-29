set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(Python3_FIND_STRATEGY LOCATION)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 QUIET)
if(NOT pybind11_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v2.13.6
    GIT_SHALLOW TRUE
    OVERRIDE_FIND_PACKAGE)
  set(PYBIND11_INSTALL ON)
  set(INCLUDE_PYBIND11 true)
  FetchContent_MakeAvailable(pybind11)
  find_package(pybind11 REQUIRED)
endif()

pybind11_add_module(pyenki enki.cpp)
include_directories(${PROJECT_SOURCE_DIR})
target_link_libraries(pyenki PRIVATE enki enkiviewer)
target_compile_definitions(pyenki
                           PRIVATE PYBIND11_DETAILED_ERROR_MESSAGES=1)

set_target_properties(pyenki PROPERTIES AUTOMOC TRUE)

install(TARGETS pyenki DESTINATION pyenki COMPONENT pyenki)

# include(CheckIPOSupported)
# check_ipo_supported(RESULT ipo_supported OUTPUT error)
# if(ipo_supported)
#   set_property(TARGET _pyenki PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
# endif()
