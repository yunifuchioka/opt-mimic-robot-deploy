#include "solo8.hpp"

#include <odri_control_interface/common.hpp>

namespace solo {

using namespace odri_control_interface;

Solo8::Solo8() {
  /**
   * Joint properties
   */
  motor_inertias_.setZero();
  motor_torque_constants_.setZero();
  joint_gear_ratios_.setZero();
  motor_max_current_.setZero();
  max_joint_torques_.setZero();
  joint_zero_positions_.setZero();

  joint_desired_torques_.setZero();
  joint_desired_positions_.setZero();
  joint_desired_velocities_.setZero();
  joint_position_gains_.setZero();
  joint_velocity_gains_.setZero();
  /**
   * Hardware status
   */
  for (unsigned i = 0; i < motor_enabled_.size(); ++i) {
    motor_enabled_[i] = false;
    motor_ready_[i] = false;
  }
  for (unsigned i = 0; i < motor_board_enabled_.size(); ++i) {
    motor_board_enabled_[0] = false;
    motor_board_errors_[0] = 0;
  }
  /**
   * Joint data
   */
  joint_positions_.setZero();
  joint_velocities_.setZero();
  joint_torques_.setZero();
  joint_sent_torques_.setZero();
  /**
   * IMU data
   */
  imu_accelerometer_.setZero();
  imu_gyroscope_.setZero();
  imu_attitude_.setZero();
  imu_linear_acceleration_.setZero();
  imu_attitude_quaternion_.setZero();
  imu_attitude_quaternion_prev_.setZero();
  /**
   * Setup some known data
   */
  // max current value:
  // https://odri.discourse.group/t/motor-current-specifications/190/2
  motor_max_current_.fill(15.0);
  motor_torque_constants_.fill(0.025);
  motor_inertias_.fill(0.045);
  joint_gear_ratios_.fill(9.0);
  /**
   * Drivers communication objects
   */
  _is_calibrating = false;
  state_ = Solo8State::initial;
}

void Solo8::initialize(const std::string& network_id) {
  // joint limit parameters TODO: set as parameters
  double max_hip_angle = M_PI * 4;
  double max_knee_angle = M_PI * 4;
  double max_joint_velocities = 160.0;
  double safety_damping = 0.2;

  // define the master board
  main_board_ptr_ = std::make_shared<MasterBoardInterface>(network_id);

  // motor indices and polarities
  VectorXi motor_numbers(8);
  motor_numbers << 0, 1, 3, 2, 5, 4, 6, 7;
  VectorXb motor_reversed(8);
  motor_reversed << true, true, false, false, true, true, false, false;

  // joint angle limits
  Eigen::VectorXd joint_lower_limits(8);
  joint_lower_limits << -max_hip_angle, -max_knee_angle, -max_hip_angle,
      -max_knee_angle, -max_hip_angle, -max_knee_angle, -max_hip_angle,
      -max_knee_angle;
  Eigen::VectorXd joint_upper_limits(8);
  joint_upper_limits << max_hip_angle, max_knee_angle, max_hip_angle,
      max_knee_angle, max_hip_angle, max_knee_angle, max_hip_angle,
      max_knee_angle;

  // define the joint module
  joints_ = std::make_shared<odri_control_interface::JointModules>(
      main_board_ptr_, motor_numbers, motor_torque_constants_(0),
      joint_gear_ratios_(0), motor_max_current_(0), motor_reversed,
      joint_lower_limits, joint_upper_limits, max_joint_velocities,
      safety_damping);

  // define the IMU
  imu_ = std::make_shared<odri_control_interface::IMU>(main_board_ptr_);

  // define the joint calibrator
  Eigen::VectorXd position_offsets(8);
  position_offsets.fill(0.);
  std::vector<odri_control_interface::CalibrationMethod> directions{
      odri_control_interface::POSITIVE, odri_control_interface::POSITIVE,
      odri_control_interface::POSITIVE, odri_control_interface::POSITIVE,
      odri_control_interface::POSITIVE, odri_control_interface::POSITIVE,
      odri_control_interface::POSITIVE, odri_control_interface::POSITIVE};
  double calib_Kp = 5.0;
  double calib_Kd = 0.05;
  double calib_T = 1.0;
  double calib_dt = 0.001;
  calib_ctrl_ = std::make_shared<odri_control_interface::JointCalibrator>(
      joints_, directions, position_offsets, calib_Kp, calib_Kd, calib_T,
      calib_dt);

  // define the robot
  robot_ = std::make_shared<odri_control_interface::Robot>(
      main_board_ptr_, joints_, imu_, calib_ctrl_);

  // initialize the robot
  robot_->Init();
}

void Solo8::acquire_sensors() {
  robot_->ParseSensorData();
  auto joints = robot_->joints;
  auto imu = robot_->imu;

  // joint data
  joint_positions_ = joints->GetPositions();
  joint_velocities_ = joints->GetVelocities();
  joint_torques_ = joints->GetMeasuredTorques();
  joint_sent_torques_ = joints->GetSentTorques();

  // imu data
  imu_linear_acceleration_ = imu->GetLinearAcceleration();
  imu_accelerometer_ = imu->GetAccelerometer();
  imu_gyroscope_ = imu->GetGyroscope();
  imu_attitude_ = imu->GetAttitudeEuler();
  // imu_attitude_quaternion_ = imu->GetAttitudeQuaternion();
  /**
   * account for mismatch between how quaternions are represented in ODRI code
   * vs RL code
   * 1. (w,x,y,z) rather than (x,y,z,w)
   * 2. Conjugate quaternion for world frame vs body frame
   * 3. Flip x axis (?) maybe this is a bug in ODRI code?
   */
  Eigen::Vector4d raw_imu_quat = imu->GetAttitudeQuaternion();
  imu_attitude_quaternion_ << raw_imu_quat(3), raw_imu_quat(0),
      -raw_imu_quat(1), -raw_imu_quat(2);

  // quaternion convention yf-08-18
  if (imu_attitude_quaternion_prev_.sum() == 0) {
    // first call to acquire_sensors
    // make sure that w >= 0
    if (imu_attitude_quaternion_[0] <= 0) {
      imu_attitude_quaternion_ *= -1;
    }
  } else {
    // if any quaternion componenent changes sign, then q <- -q
    for (unsigned int quat_idx = 0; quat_idx < 4; quat_idx++) {
      if (abs(imu_attitude_quaternion_[quat_idx] -
              imu_attitude_quaternion_prev_[quat_idx]) > 1.0) {
        imu_attitude_quaternion_ *= -1;
        continue;
      }
    }
  }

  // // zero-yaw filter
  // // assumes that large yaw angle variations don't occur
  // if (imu_attitude_quaternion_.head(3).norm() < 10e-6) {
  //   imu_attitude_quaternion_ << 1.0, 0.0, 0.0, 0.0;
  // } else {
  //   imu_attitude_quaternion_(3) = 0.0;
  //   // imu_attitude_quaternion_.head(3).normalize();
  // }

  // // zero-roll filter
  // // assumes that large yaw angle variations don't occur
  // if (imu_attitude_quaternion_.head(3).norm() < 10e-6) {
  //   imu_attitude_quaternion_ << 1.0, 0.0, 0.0, 0.0;
  // } else {
  //   imu_attitude_quaternion_(1) = 0.0;
  //   imu_attitude_quaternion_.head(3).normalize();
  // }


  // motor status
  ConstRefVectorXb motor_enabled = joints->GetEnabled();
  ConstRefVectorXb motor_ready = joints->GetReady();
  for (int i = 0; i < 8; i++) {
    motor_enabled_[i] = motor_enabled[i];
    motor_ready_[i] = motor_ready[i];
  }

  // motor board status
  ConstRefVectorXi motor_board_errors = joints->GetMotorDriverErrors();
  ConstRefVectorXb motor_driver_enabled = joints->GetMotorDriverEnabled();
  for (int i = 0; i < 4; i++) {
    motor_board_errors_[i] = motor_board_errors[i];
    motor_board_enabled_[i] = motor_driver_enabled[i];
  }

  // set imu quaternion memory for detecting wrapping
  imu_attitude_quaternion_prev_ << imu_attitude_quaternion_;
}

void Solo8::send_joint_commands() {
  robot_->joints->SetTorques(joint_desired_torques_);
  robot_->joints->SetDesiredPositions(joint_desired_positions_);
  robot_->joints->SetDesiredVelocities(joint_desired_velocities_);
  robot_->joints->SetPositionGains(joint_position_gains_);
  robot_->joints->SetVelocityGains(joint_velocity_gains_);

  switch (state_) {
    case Solo8State::initial:
      robot_->joints->SetZeroCommands();
      if (!robot_->IsTimeout() && !robot_->IsAckMsgReceived()) {
        robot_->SendInit();
      } else if (!robot_->IsReady()) {
        robot_->SendCommand();
      } else {
        state_ = Solo8State::ready;
      }
      break;

    case Solo8State::ready:
      if (calibrate_request_) {
        calibrate_request_ = false;
        state_ = Solo8State::calibrate;
        _is_calibrating = true;
        robot_->joints->SetZeroCommands();
      }
      robot_->SendCommand();
      break;

    case Solo8State::calibrate:
      if (calib_ctrl_->Run()) {
        state_ = Solo8State::ready;
        _is_calibrating = false;
      }
      robot_->SendCommand();
      break;
  }
}

bool Solo8::request_calibration(const Vector8d& home_offset_rad) {
  printf("Solo8::request_calibration called\n");
  Eigen::VectorXd hor = home_offset_rad;
  calib_ctrl_->UpdatePositionOffsets(hor);
  calibrate_request_ = true;
  return true;
}

}  // namespace solo
