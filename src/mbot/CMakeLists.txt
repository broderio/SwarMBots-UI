cmake_minimum_required(VERSION 3.5)
project(mbot C CXX)

set(SOURCE_FILES
    mbot.h
    mbot.cpp
)

add_library(mbot STATIC ${SOURCE_FILES})

install(TARGETS mbot DESTINATION ${INSTALL_LIB_DIR})
install(FILES mbot.h DESTINATION ${INSTALL_INCLUDE_DIR})