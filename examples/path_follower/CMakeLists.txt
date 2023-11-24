include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/mbot)
add_executable(path_follower main.cpp)
find_package(websocketpp CONFIG REQUIRED)
target_link_libraries(path_follower mbot websocketpp::websocketpp)