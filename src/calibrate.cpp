/**
 * Script used to calibrate the robot
 */

#include "common.hpp"
#include "main.hpp"
#include "solo8.hpp"

using namespace solo;

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* thread_data_void_ptr) {
  ThreadCalibrationData* thread_data_ptr =
      (static_cast<ThreadCalibrationData*>(thread_data_void_ptr));
  std::shared_ptr<Solo8> robot = thread_data_ptr->robot;

  Vector8d eight_zeros;
  eight_zeros.setZero();

  robot->request_calibration(eight_zeros);

  size_t count = 0;

  rt_printf("Move the robot to zero position. Press Ctrl-c when done \n");
  while (!CTRL_C_DETECTED) {
    robot->acquire_sensors();

    robot->set_joint_desired_torques(0.0);
    robot->send_joint_commands();

    if ((count % 100) == 0) {
      //   print_vector("Home offset angle [Rad]",
      //   -robot->get_joint_positions());
      saveData("../config/calib_data.csv", -robot->get_joint_positions());
    }

    real_time_tools::Timer::sleep_sec(0.001);
    ++count;
  }
}

int main(int argc, char** argv) {
  real_time_tools::RealTimeThread thread;
  enable_ctrl_c();

  if (argc != 2) {
    throw std::runtime_error(
        "Please provide the interface name (i.e. using 'ifconfig' on linux)");
  }

  std::shared_ptr<Solo8> robot = std::make_shared<Solo8>();
  robot->initialize(std::string(argv[1]));

  ThreadCalibrationData thread_data(robot);

  rt_printf("Press enter to start calibration \n");
  char str[256];
  std::cin.get(str, 256);  // get c-string

  thread.create_realtime_thread(&control_loop, &thread_data);

  rt_printf("control loop started \n");

  while (!CTRL_C_DETECTED) {
    real_time_tools::Timer::sleep_sec(0.01);
  }

  thread.join();

  return 0;
}
