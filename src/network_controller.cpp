#include "network_controller.hpp"

using namespace solo;

void NetworkController::initialize_network(const std::string filename) {
  // load neural network model
  try {
    std::string model_dir = "../models/";
    network_ = torch::jit::load(model_dir + filename + "_script.pt");
  } catch (const c10::Error& e) {
    std::cerr << "error loading neural network model\n";
  }
}

void NetworkController::calc_control() {
  // find reference trajectory index corresponding to current time
  int traj_idx = (int)((time_ - time_stamp_state_change_) * ref_traj_max_idx_ /
                       ref_traj_max_time_);
  traj_idx = traj_idx % ref_traj_max_idx_;

  // set desired position to reference according to residual policy
  setReferenceMotionTraj(traj_idx);

  // get sensor data
  Vector8d joint_positions = robot_->get_joint_positions();
  Vector8d joint_velocities = robot_->get_joint_velocities();
  Eigen::Vector4d imu_attitude_quaternion =
      robot_->get_imu_attitude_quaternion();

  // calculate phase according to time
  double phase =
      2.0 * M_PI / ref_traj_max_time_ * (time_ - time_stamp_state_change_);

  // construct network input (ie RL observation)
  VectorObservation observation;
  // observation << imu_attitude_quaternion, joint_positions,
  // filtered_velocity_,
  //     sensor_history_, action_history_, cos(phase), sin(phase);
  observation << imu_attitude_quaternion, joint_positions, filtered_velocity_,
      cos(phase), sin(phase);

  // // uncomment to print observation components
  // std::cout << observation.segment(0, 4).transpose() << std::endl;
  // std::cout << observation.segment(4, 8).transpose() << std::endl;
  // std::cout << observation.segment(12, 8).transpose() << std::endl;
  // std::cout << observation.segment(20, 4).transpose() << std::endl;
  // std::cout << observation.segment(24, 8).transpose() << std::endl;
  // std::cout << observation.segment(32, 8).transpose() << std::endl;
  // std::cout << observation.segment(40, 4).transpose() << std::endl;
  // std::cout << observation.segment(44, 8).transpose() << std::endl;
  // std::cout << observation.segment(52, 8).transpose() << std::endl;
  // std::cout << observation.segment(60, 4).transpose() << std::endl;
  // std::cout << observation.segment(64, 8).transpose() << std::endl;
  // std::cout << observation.segment(72, 8).transpose() << std::endl;
  // std::cout << observation.segment(80, 8).transpose() << std::endl;
  // std::cout << observation.segment(88, 8).transpose() << std::endl;
  // std::cout << observation.segment(96, 8).transpose() << std::endl;
  // std::cout << observation.segment(104, 2).transpose() << std::endl <<
  // std::endl;

  // convert Eigen double vector to torch double tensor. Note the matrix
  // transpose according to the conventions of Eigen and Torch
  torch::Tensor input_tensor =
      torch::from_blob(observation.data(), {1, NETWORK_INPUT_DIM}, at::kDouble)
          .clone();

  // convert torch double tensor to torchscript float IValue
  std::vector<torch::jit::IValue> input_ivalue;
  input_ivalue.push_back(input_tensor.to(torch::kFloat));

  // evaluate network
  torch::Tensor output_tensor =
      network_.forward(input_ivalue).toTuple()->elements()[0].toTensor();

  // convert network output to Eigen double vector. Note the matrix
  // transpose according to the conventions of Eigen and Torch
  VectorAction output(output_tensor.to(torch::kDouble).data_ptr<double>());

  // update sensor history for the next control step
  sensor_history_.tail((HORIZON - 1) * SENSOR_DIM)
      << sensor_history_.head((HORIZON - 1) * SENSOR_DIM).eval();
  sensor_history_.head(SENSOR_DIM) << imu_attitude_quaternion, joint_positions,
      filtered_velocity_;

  action_history_.tail((HORIZON - 1) * ACTION_DIM)
      << action_history_.head((HORIZON - 1) * ACTION_DIM).eval();
  action_history_.head(ACTION_DIM) << output;

  // depending on state, set joint commands accordingly
  // NOTE: even though network output is only used for motion state, we always
  // need to perform above calculation, otherwise a communication timeout error
  // occurs. Maybe this has to do with when the torch tensor memory gets
  // allocated? (ie during initialization vs during control loop)
  switch (controllerState_) {
    case ControllerState::homing:
      desired_positions_.setZero();
      desired_velocities_.setZero();
      desired_torques_.setZero();

      // FSM state transition if homing procedure is finished
      if (robot_->isReady()) {
        controllerState_ = ControllerState::stand;
        // controllerState_ = ControllerState::fold;
        time_stamp_state_change_ = time_;
      }
      break;

    case ControllerState::fold:
      // set desired pose according legs being folded
      desired_positions_ << M_PI / 2.0, M_PI, M_PI / 2.0, M_PI, -M_PI / 2.0,
          M_PI, -M_PI / 2.0, M_PI;

      // warm start
      desired_positions_ =
          desired_positions_ *
          std::min(1.0, std::max(time_ - time_stamp_state_change_, 0.0));

      break;

    case ControllerState::stand:
      // set desired pose according to initial pose of reference trajectory
      setReferenceMotionTraj(0);
      desired_positions_ << desired_positions_reference_;

      // warm start
      desired_positions_ =
          desired_positions_ *
          std::min(1.0, std::max(time_ - time_stamp_state_change_, 0.0));

      // FSM state transition after some time, for the user to put the robot on
      // the ground
      if (time_ - time_stamp_state_change_ > 10.0) {
        controllerState_ = ControllerState::motion;
        time_stamp_state_change_ = time_;
      }

      break;

    case ControllerState::motion:

      // // temporary
      // if ((time_ - time_stamp_state_change_) < ref_traj_max_time_) {
      //   // set joint targets according to residual neural network policy
      //   desired_positions_ = desired_positions_reference_;
      //   desired_positions_ += output;
      //   // desired_velocities_ = desired_velocities_reference_;
      //   desired_torques_ = desired_torques_reference_;
      // }

      // set joint targets according to residual neural network policy
      desired_positions_ = desired_positions_reference_;
      desired_positions_ += output;
      // desired_velocities_ = desired_velocities_reference_;
      desired_torques_ = desired_torques_reference_;

      break;
  }
}

void NetworkController::setReferenceMotionTraj(const int traj_idx) {
  // Eigen::Vector3d base_pos;
  // Eigen::Vector4d base_quat;
  Eigen::Matrix<double, 8, 1> desired_joint_position;
  Eigen::Matrix<double, 8, 1> desired_joint_velocity;
  Eigen::Matrix<double, 8, 1> desired_joint_torque;

  Eigen::Matrix<double, 38, 1> traj_t;
  traj_t << ref_traj_.row(traj_idx).transpose();
  desired_joint_position << traj_t.segment(14, 8);
  desired_joint_velocity << traj_t.segment(22, 8);
  desired_joint_torque << traj_t.segment(30, 8);

  desired_positions_reference_ = desired_joint_position;
  desired_velocities_reference_ = desired_joint_velocity;
  desired_torques_reference_ = desired_joint_torque;
}