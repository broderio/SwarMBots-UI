add_library(comms "")

target_sources(comms
  PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/comms.c
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/comms.h
)

target_include_directories(comms
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)