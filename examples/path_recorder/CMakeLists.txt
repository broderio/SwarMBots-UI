include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/mbot)
add_executable(path_recorder main.cpp)
find_package(websocketpp CONFIG REQUIRED)
target_link_libraries(path_recorder mbot websocketpp::websocketpp)