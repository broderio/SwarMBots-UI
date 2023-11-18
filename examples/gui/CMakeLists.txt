include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/mbot)
add_executable(gui main.cpp)
find_package(websocketpp CONFIG REQUIRED)
target_link_libraries(gui mbot websocketpp::websocketpp)