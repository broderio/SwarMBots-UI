include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/mbot)
add_executable(drive_triangle main.cpp)
target_link_libraries(drive_triangle mbot)