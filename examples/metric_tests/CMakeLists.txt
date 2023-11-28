include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/mbot)
add_executable(metric_tests main.cpp)
find_package(websocketpp CONFIG REQUIRED)
target_link_libraries(metric_tests mbot websocketpp::websocketpp)