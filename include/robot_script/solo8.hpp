/**
 * Class that represents the hardware Solo 8 robot
 * This is mostly copy+pasted from open dynamic robot initiative solo
 * https://raw.githubusercontent.com/open-dynamic-robot-initiative/solo/master/include/solo/solo8.hpp
 */

#pragma once

#include <odri_control_interface/calibration.hpp>
#include <odri_control_interface/robot.hpp>

#include "common.hpp"

namespace solo {

enum Solo8State { initial, ready, calibrate };

class Solo8 {
 public:
  /**
   * @brief constructor to initialize values for variables
   */
  Solo8();

  /**
   * @brief initialize the robot by setting aligning the motors and calibrate
   * the sensors to 0
   */
  void initialize(const std::string& network_id);

  /**
   * @brief acquire_sensors acquire all available sensors, WARNING !!!!
   * this method has to be called prior to any getter to have up to date data.
   */
  void acquire_sensors();

  /**
   * @brief send_joint_commands sends the sends desired PD+torque targets to
   * motors
   */
  void send_joint_commands();

  /**
   * @brief Asynchrounous calibrate the joints by moving to the next joint index
   * position.
   *
   * @param home_offset_rad This is the angle between the index and the zero
   * pose.
   * @return true
   * @return false
   */
  bool request_calibration(const Vector8d& home_offset_rad);

  /**
   * Sensor Data
   */

  /**
   * @brief get_joint_positions
   * @return  the joint angle of each module
   * WARNING !!!!
   * The method <acquire_sensors>"()" has to be called
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Vector8d> get_joint_positions() { return joint_positions_; }

  /**
   * @brief get_joint_velocities
   * @return the joint velocities
   * WARNING !!!!
   * The method <acquire_sensors>"()" has to be called
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Vector8d> get_joint_velocities() {
    return joint_velocities_;
  }

  /**
   * @brief get_joint_torques
   * @return the joint torques
   * WARNING !!!!
   * The method <acquire_sensors>"()" has to be called
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Vector8d> get_joint_torques() { return joint_torques_; }

  /**
   * @brief get_joint_torques
   * @return the sent joint torques
   * WARNING !!!!
   * The method <acquire_sensors>"()" has to be called
   * prior to any getter to have up to date data.

   */
  const Eigen::Ref<Vector8d> get_joint_sent_torques() {
    return joint_sent_torques_;
  }

  /**
   * @brief get_imu_accelerometer
   * @return  the imu accelerometer reading
   * WARNING !!!!
   * The method <acquire_sensors>"()" has to be called
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Eigen::Vector3d> get_imu_accelerometer() {
    return imu_accelerometer_;
  }

  /**
   * @brief get_imu_gyroscope
   * @return  the imu gyroscope reading
   * WARNING !!!!
   * The method <acquire_sensors>"()" has to be called
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Eigen::Vector3d> get_imu_gyroscope() {
    return imu_gyroscope_;
  }

  /**
   * @brief get_imu_attitude
   * @return  the imu attitude
   * WARNING !!!!
   * The method <acquire_sensors>"()" has to be called
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Eigen::Vector3d> get_imu_attitude() { return imu_attitude_; }

  /**
   * @brief get_imu_linear_acceleration
   * @return  the imu linear acceleration (ie gravity subtracted out)
   * WARNING !!!!
   * The method <acquire_sensors>"()" has to be called
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Eigen::Vector3d> get_imu_linear_acceleration() {
    return imu_linear_acceleration_;
  }

  /**
   * @brief get_imu_attitude_quaternion
   * @return  the imu attitude represented in a quaternion
   * WARNING !!!!
   * The method <acquire_sensors>"()" has to be called
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Eigen::Vector4d> get_imu_attitude_quaternion() {
    return imu_attitude_quaternion_;
  }

  /**
   * Setter functions for joints, input is Eigen vector
   */
  void set_joint_desired_torques(const Vector8d input_vector) {
    joint_desired_torques_ = input_vector;
  }
  void set_joint_desired_positions(const Vector8d input_vector) {
    joint_desired_positions_ = input_vector;
  }
  void set_joint_desired_velocities(const Vector8d input_vector) {
    joint_desired_velocities_ = input_vector;
  }
  void set_joint_position_gains(const Vector8d input_vector) {
    joint_position_gains_ = input_vector;
  }
  void set_joint_velocity_gains(const Vector8d input_vector) {
    joint_velocity_gains_ = input_vector;
  }
  /**
   * Setter functions for joints, input is double
   */
  void set_joint_desired_torques(const double input_double) {
    joint_desired_torques_.fill(input_double);
  }
  void set_joint_desired_positions(const double input_double) {
    joint_desired_positions_.fill(input_double);
  }
  void set_joint_desired_velocities(const double input_double) {
    joint_desired_velocities_.fill(input_double);
  }
  void set_joint_position_gains(const double input_double) {
    joint_position_gains_.fill(input_double);
  }
  void set_joint_velocity_gains(const double input_double) {
    joint_velocity_gains_.fill(input_double);
  }

  /**
   * @brief custom function for determining whether robot has finished its
   * homing (or "calibration" as ODRI code calls it) sequence
   *
   * @return true
   * @return false
   */
  bool isReady() {
    return state_ == 1 && _is_calibrating == 0 && calibrate_request_ == 0;
  }

 private:
  /**
   * Joint properties
   */
  Vector8d motor_inertias_;         /**< motors inertia. */
  Vector8d motor_torque_constants_; /**< DCM motor torque constants. */
  Vector8d joint_gear_ratios_;      /**< joint gear ratios (9). */
  Vector8d motor_max_current_;      /**< Max appliable current before the robot
                                       shutdown. */
  Vector8d joint_zero_positions_;   /**< Offset to the theoretical "0" pose. */
  Vector8d max_joint_torques_;      /**< Max joint torques (Nm) */

  /**
   * Hardware status
   */
  /**
   * @brief This gives the status (enabled/disabled) of each motors using the
   * joint ordering convention.
   */
  std::array<bool, 8> motor_enabled_;
  /**
   * @brief This gives the status (enabled/disabled) of each motors using the
   * joint ordering convention.
   */
  std::array<bool, 8> motor_ready_;
  /**
   * @brief This gives the status (enabled/disabled of the onboard control
   * cards).
   */
  std::array<bool, 4> motor_board_enabled_;
  /**
   * @brief This gives the status (enabled/disabled of the onboard control
   * cards).
   */
  std::array<int, 4> motor_board_errors_;

  /**
   * Joint data
   */
  Vector8d joint_positions_;
  Vector8d joint_velocities_;
  Vector8d joint_torques_;
  Vector8d joint_sent_torques_;

  Vector8d joint_desired_torques_;
  Vector8d joint_desired_positions_;
  Vector8d joint_desired_velocities_;
  Vector8d joint_position_gains_;
  Vector8d joint_velocity_gains_;

  /**
   * IMU data
   */
  Eigen::Vector3d imu_accelerometer_;
  Eigen::Vector3d imu_gyroscope_;
  Eigen::Vector3d imu_attitude_;
  Eigen::Vector3d imu_linear_acceleration_;
  Eigen::Vector4d imu_attitude_quaternion_;
  Eigen::Vector4d imu_attitude_quaternion_prev_;

  /**
   * Drivers communication objects
   */
  /**
   * @brief Main board drivers.
   * PC <- Ethernet/Wifi -> main board <- SPI -> Motor Board
   */
  std::shared_ptr<MasterBoardInterface> main_board_ptr_;
  /** @brief Collection of Joints for solo 8 */
  std::shared_ptr<odri_control_interface::JointModules> joints_;
  /** @brief Robot Imu drivers. */
  std::shared_ptr<odri_control_interface::IMU> imu_;
  /** @brief Controller to run the calibration procedure */
  std::shared_ptr<odri_control_interface::JointCalibrator> calib_ctrl_;
  /** @brief The odri robot abstraction  */
  std::shared_ptr<odri_control_interface::Robot> robot_;

  /**
   * State variables
   */
  /** @brief If the joint calibration is active or not. */
  bool _is_calibrating;
  /** @brief Indicator if calibration should start. */
  bool calibrate_request_;
  /** @brief State of the solo robot. */
  Solo8State state_;
};

}  // namespace solo