cmake_minimum_required(VERSION 3.5)
project(comms C CXX)

set(SOURCE_FILES
    comms.h
    comms.c
)

add_library(comms STATIC ${SOURCE_FILES})

install(TARGETS comms DESTINATION ${INSTALL_LIB_DIR})
install(FILES comms.h DESTINATION ${INSTALL_INCLUDE_DIR})
