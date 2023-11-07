add_library(mbot "")

target_sources(mbot
  PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/mbot.cpp
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/mbot.hpp
)

target_include_directories(mbot
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)