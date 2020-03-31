include_directories (${PROJECT_SOURCE_DIR})
link_directories (${PROJECT_BINARY_DIR})

add_executable(enkiMinimalThymio enkiMinimalThymio.cpp)
target_link_libraries(enkiMinimalThymio enki)
