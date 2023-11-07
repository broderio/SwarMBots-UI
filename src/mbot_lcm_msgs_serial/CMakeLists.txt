add_library(mbot_lcm_msgs_serial "")

target_sources(mbot_lcm_msgs_serial
  PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/mbot_lcm_msgs_serial.c
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/mbot_lcm_msgs_serial.h
)

target_include_directories(mbot_lcm_msgs_serial
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)