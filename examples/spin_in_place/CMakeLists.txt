include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/mbot)
add_executable(spin_in_place main.cpp)
find_package(websocketpp CONFIG REQUIRED)
target_link_libraries(spin_in_place mbot websocketpp::websocketpp)