include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/mbot)
add_executable(p_controller main.cpp)
target_link_libraries(p_controller mbot)