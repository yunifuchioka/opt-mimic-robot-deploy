#include "main.hpp"

#include "common.hpp"
#include "imu_controller.hpp"
#include "network_controller.hpp"
#include "phase_controller.hpp"
#include "solo8.hpp"

using namespace solo;

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* thread_data_void_ptr) {
  ThreadCalibrationData* thread_data_ptr =
      (static_cast<ThreadCalibrationData*>(thread_data_void_ptr));
  std::shared_ptr<Solo8> robot = thread_data_ptr->robot;

  double dt_des = 0.001;
  double kp = 3.0;
  double kd = 0.3;

  Vector8d joint_desired_positions;
  Vector8d joint_desired_velocities;
  Vector8d joint_desired_torques;

  robot->acquire_sensors();

  // Calibrates the robot.
  Vector8d joint_index_to_zero = thread_data_ptr->joint_index_to_zero;
  robot->request_calibration(joint_index_to_zero);

  robot->set_joint_position_gains(kp);
  robot->set_joint_velocity_gains(kd);

  size_t count = 0;
  double t = 0.0;

  Eigen::MatrixXd ref_traj;
  // ref_traj = openData("../traj/08-03-stand.csv");
  // ref_traj = openData("../traj/08-03-squat.csv");
  // ref_traj = openData("../traj/08-19-trot.csv");
  // ref_traj = openData("../traj/08-19-front-hop.csv");
  // ref_traj = openData("../traj/08-19-180-backflip.csv");
  // ref_traj = openData("../traj/08-29-jump.csv");
  // ref_traj = openData("../traj/08-24-biped-step.csv");
  // ref_traj = openData("../traj/08-31-jump.csv");
  // ref_traj = openData("../traj/09-01-jump.csv");
  ref_traj = openData("../traj/09-08-biped-step.csv");

  NetworkController controller(robot);
  controller.set_traj(ref_traj);
  // controller.initialize_network("08-26-trot");
  // controller.initialize_network("08-31-front-hop");
  // controller.initialize_network("08-30-jump-1");
  // controller.initialize_network("08-31-biped-zhaoming");
  // controller.initialize_network("08-31-jump");
  // controller.initialize_network("09-01-front-hop");
  // below are history conditioned policies
  // controller.initialize_network("09-02-trot");
  // controller.initialize_network("09-01-front-hop-2");
  // controller.initialize_network("09-01-jump");
  // controller.initialize_network("09-01-biped-step");
  // controller.initialize_network("09-03-trot-torque-position");
  // controller.initialize_network("09-02-front-hop");
  // controller.initialize_network("09-03-front-hop-torque-noise-2");
  // controller.initialize_network("09-04-front-hop-offset-only");
  // controller.initialize_network("09-03-front-hop-offset");
  // controller.initialize_network("09-04-front-hop-imu-drift-2");
  // controller.initialize_network("09-06-trot");
  // controller.initialize_network("09-05-front-hop");
  // controller.initialize_network("09-05-zhaoming_biped");
  // below are non-history conditioned policies
  // controller.initialize_network("09-07-trot-no-hist-less-action-torque");
  // controller.initialize_network("09-06-no-hist-less-action-torque");
  // controller.initialize_network("09-06-biped-step-no-history");
  // controller.initialize_network("09-07-front-hop-no-hist-less-action-torque");
  // controller.initialize_network("09-07-zhaoming_biped_iter69000");
  // controller.initialize_network("09-07-biped-step-impedance-kd05");
  // controller.initialize_network("09-07-biped-step-impedance-more-action-torque");
  // controller.initialize_network("09-08-front-hop-impedance-iter10000");
  // controller.initialize_network("09-08-front-hop-impedance");
  // controller.initialize_network("09-08-front-hop-impedance-torque");
  // controller.initialize_network("09-10-trot-pos-torque");
  // controller.initialize_network("09-08-biped-step-feedforward");
  // controller.initialize_network("09-09-180-backflip-pos-torque");

  // controller.initialize_network("09-10-front-hop-pos-iter65500");
  // controller.initialize_network("09-10-front-hop-pos-torque-iter65500");
  // controller.initialize_network("09-10-front-hop-pos-vel-torque-iter65500");

  // controller.initialize_network("09-12-180-backflip-seed2-iter66000");
  // controller.initialize_network("09-12-180-backflip-pos-torque-seed3-iter66000");
  // controller.initialize_network("09-12-180-backflip-pos-vel-torque-seed2-iter66000");

  // controller.initialize_network("09-10-biped-step-seed3-iter93500");
  // controller.initialize_network("09-10-biped-step-pos-torque-iter93500");
  // controller.initialize_network("09-10-biped-step-pos-vel-torque-iter93500");

  // controller.initialize_network("final-trot-p");
  // controller.initialize_network("final-trot-pt");
  // controller.initialize_network("final-trot-pvt");
  // controller.initialize_network("final-front-hop-pt");
  // controller.initialize_network("final-front-hop-pt-iter60k");
  // controller.initialize_network("final-front-hop-pt-iter70k");
  // controller.initialize_network("final-180-backflip-pt");
  // controller.initialize_network("final-biped-step-pt-iter40k");
  controller.initialize_network("final-biped-step-pt-iter50k");
  // controller.initialize_network("final-biped-step-pt-iter60k");
  // controller.initialize_network("final-biped-step-pt-iter70k");
  // controller.initialize_network("final-biped-step-pt");

  // buffer for storing joint velocity values for filtering
  // length of 20 corresponds to RL policy frequency
  Eigen::MatrixXd vel_buffer(8, 20);
  vel_buffer.setZero();
  unsigned int buffer_counter = 0;
  Vector8d filtered_velocity;
  filtered_velocity.setZero();

  // PhaseController controller(robot);
  // controller.set_motion_type(PhaseController::squat);
  // double period = 0.8;   // for squat
  // double period = 8.16;  // for walk

  auto tic = Clock::now();
  real_time_tools::Timer::sleep_sec(dt_des);
  real_time_tools::Timer::sleep_sec(dt_des);

  Eigen::VectorXd log_vec(53);  // vector to print when logging to csv

  while (!CTRL_C_DETECTED) {
    robot->acquire_sensors();
    t = std::chrono::duration<double>(Clock::now() - tic).count();

    if (count % 20 == 0) {  // control_dt = 0.02 in RL code

      // specify filtered velocity to controller to be input into network
      filtered_velocity = vel_buffer.rowwise().mean();
      controller.set_filtered_velocity(filtered_velocity);

      // update network policy
      controller.set_time(t);
      controller.calc_control();
      joint_desired_positions = controller.get_desired_positions();
      joint_desired_velocities = controller.get_desired_velocities();
      joint_desired_torques = controller.get_desired_torques();

      // reset velocity filter buffer
      vel_buffer.setZero();
      buffer_counter = 0;
    }

    // send desired PD+torque targets to robot
    robot->set_joint_desired_positions(joint_desired_positions);
    robot->set_joint_desired_velocities(joint_desired_velocities);
    robot->set_joint_desired_torques(joint_desired_torques);
    robot->send_joint_commands();

    // store velocity in buffer for filtering
    vel_buffer.col(buffer_counter) << robot->get_joint_velocities();
    buffer_counter++;

    if ((count % 1) == 0) {
      log_vec(0) = t;
      log_vec.segment(1, 4) = robot->get_imu_attitude_quaternion();
      log_vec.segment(5, 8) = robot->get_joint_positions();
      log_vec.segment(13, 8) = filtered_velocity;
      // log_vec.segment(13, 8) = robot->get_joint_velocities();
      log_vec.segment(21, 8) = robot->get_joint_torques();
      log_vec.segment(29, 8) = joint_desired_positions;
      log_vec.segment(37, 8) = joint_desired_velocities;
      log_vec.segment(45, 8) = joint_desired_torques;
      print_vector_csv(log_vec);

      // uncomment to get delta time rather than absolute time
      // tic = Clock::now();
    }

    real_time_tools::Timer::sleep_sec(dt_des);
    ++count;
  }  // endwhile
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

  rt_printf("Press enter to start the control loop \n");
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
