set(SOURCES mbot.cpp)
add_library(mbot ${SOURCES})

find_package(websocketpp CONFIG REQUIRED)
target_link_libraries(mbot PRIVATE websocketpp::websocketpp)