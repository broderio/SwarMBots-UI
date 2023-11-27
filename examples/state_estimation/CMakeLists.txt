include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/mbot)
add_executable(state_estimation main.cpp)
find_package(websocketpp CONFIG REQUIRED)
target_link_libraries(state_estimation mbot websocketpp::websocketpp)