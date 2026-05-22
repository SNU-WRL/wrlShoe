cmake_minimum_required(VERSION 3.16)
project(motorized_shoe_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

add_executable(motorized_shoe_app
  src/main.cpp
  src/can_utils.cpp
  src/canopen_utils.cpp
  src/gait_fsm.cpp
  src/config.cpp
  src/data_logger.cpp
  src/keyboard_input.cpp
  src/read_can_interpret_imu_node.cpp
  src/gait_phase_detection_node.cpp
  src/send_can_command_to_elmo_node.cpp
  src/read_can_malfunction_from_elmo_node.cpp
)

target_link_libraries(motorized_shoe_app pthread)

add_executable(motorized_shoe_imu_gait_app
  src/main_imu_gait.cpp
  src/can_utils.cpp
  src/gait_fsm.cpp
  src/config.cpp
  src/data_logger.cpp
  src/read_can_interpret_imu_node.cpp
  src/gait_phase_detection_node.cpp
)

target_link_libraries(motorized_shoe_imu_gait_app pthread)

add_executable(motorized_shoe_slip_app
  src/main_slip.cpp
  src/can_utils.cpp
  src/canopen_utils.cpp
  src/gait_fsm.cpp
  src/config.cpp
  src/data_logger.cpp
  src/keyboard_input.cpp
  src/read_can_interpret_imu_node.cpp
  src/gait_phase_detection_node.cpp
  src/send_can_command_to_elmo_node.cpp
  src/read_can_malfunction_from_elmo_node.cpp
  src/slip_perturbation_node.cpp
)

target_link_libraries(motorized_shoe_slip_app pthread)

add_executable(send_zero_velocity_to_elmo
  src/send_zero_velocity_to_elmo.cpp
  src/can_utils.cpp
  src/canopen_utils.cpp
)

target_link_libraries(send_zero_velocity_to_elmo pthread)

add_executable(disable_elmo_drive
  src/disable_elmo_drive.cpp
  src/can_utils.cpp
  src/canopen_utils.cpp
)

target_link_libraries(disable_elmo_drive pthread)
