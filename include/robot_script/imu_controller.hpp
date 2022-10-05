/**
 * controller that uses only IMU data
 */

#pragma once

#include "controller.hpp"

class IMUController : public Controller {
 public:
  enum MotionType { stand, tilt_balance };

  /**
   * constructor calls the parent controller constructor, then initializes its
   * own internal variables
   */
  IMUController(const std::shared_ptr<Solo8>& robot) : Controller{robot} {
    motion_type_ = MotionType::stand;
  }

  /**
   * Calculates control based on robot IMU data
   *
   * WARNING !!!!
   * The method robot_->acquire_sensors() has to be called prior to this
   * function call in order to compute the control with up to date sensor data
   */
  void calc_control();

  /**
   * setters for private variables
   */
  void set_motion_type(MotionType motion_type) { motion_type_ = motion_type; };

 private:
  MotionType motion_type_;
};