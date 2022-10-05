cmake_minimum_required(VERSION 3.5)
project(robot_script)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(mpi_cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(real_time_tools REQUIRED)
find_package(odri_control_interface REQUIRED)
find_package(Torch REQUIRED)

add_library(solo8 src/solo8.cpp)
target_link_libraries(solo8
  Eigen3::Eigen
  odri_control_interface::odri_control_interface)

add_library(phase_controller src/phase_controller.cpp)
target_link_libraries(phase_controller
  Eigen3::Eigen
  odri_control_interface::odri_control_interface)

add_library(imu_controller src/imu_controller.cpp)
target_link_libraries(imu_controller
  Eigen3::Eigen
  odri_control_interface::odri_control_interface)

add_library(network_controller src/network_controller.cpp)
target_link_libraries(network_controller
  Eigen3::Eigen
  "${TORCH_LIBRARIES}"
  odri_control_interface::odri_control_interface)

include_directories(
  include/robot_script
  # Set manually because Eigen sets a non standard INCLUDE DIR
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(main src/main.cpp)
target_link_libraries(main
  solo8
  phase_controller
  imu_controller
  network_controller
  Eigen3::Eigen
  real_time_tools::real_time_tools
  odri_control_interface::odri_control_interface)

add_executable(calibrate src/calibrate.cpp)
target_link_libraries(calibrate
  solo8
  Eigen3::Eigen
  real_time_tools::real_time_tools
  odri_control_interface::odri_control_interface)

add_executable(test_torch src/test_torch.cpp)
target_link_libraries(test_torch
  solo8
  network_controller)

install(TARGETS
  main
  calibrate
  test_torch
  DESTINATION lib/${PROJECT_NAME})

ament_package()
