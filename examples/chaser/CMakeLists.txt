include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/mbot)
add_executable(chaser main.cpp)
find_package(websocketpp CONFIG REQUIRED)
target_link_libraries(chaser mbot websocketpp::websocketpp)