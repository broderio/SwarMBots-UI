add_executable(main main.cpp)

add_subdirectory(comms)
add_subdirectory(mbot)
add_subdirectory(mbot_lcm_msgs_serial)

target_link_libraries(main
  PRIVATE
    comms
    mbot
    mbot_lcm_msgs_serial
)