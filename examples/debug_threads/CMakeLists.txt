include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/mbot)
add_executable(debug_threads main.cpp)
target_link_libraries(debug_threads mbot)