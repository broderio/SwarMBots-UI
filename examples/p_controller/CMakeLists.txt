include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/mbot)
add_executable(p_controller main.cpp)
find_package(websocketpp CONFIG REQUIRED)
target_link_libraries(p_controller mbot websocketpp::websocketpp)