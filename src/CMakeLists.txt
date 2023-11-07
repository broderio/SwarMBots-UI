cmake_minimum_required(VERSION 3.2)
project(swarmbots-main)

add_subdirectory(comms)
add_subdirectory(mbot)
set(SOURCE_FILES main.cpp)

add_executable(swarmbots-main ${SOURCE_FILES})
target_link_libraries(swarmbots-main comms mbot)
install(TARGETS swarmbots-main DESTINATION ${INSTALL_BIN_DIR})
