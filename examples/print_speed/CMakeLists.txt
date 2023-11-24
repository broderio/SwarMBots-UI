include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/mbot)
add_executable(print_speed main.cpp)
find_package(websocketpp CONFIG REQUIRED)
target_link_libraries(print_speed mbot websocketpp::websocketpp)