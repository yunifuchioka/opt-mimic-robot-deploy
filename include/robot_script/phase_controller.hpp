/**
 * controller that uses only phase information as input (ie no robot sensor
 * data)
 */

#pragma once

#include "controller.hpp"

class PhaseController : public Controller {
 public:
  enum MotionType { stand, squat, tilt_body, step_in_place, walk };

  /**
   * constructor calls the parent controller constructor, then initializes its
   * own internal variables
   */
  PhaseController(const std::shared_ptr<Solo8>& robot) : Controller{robot} {
    phase_ = 0;
    motion_type_ = MotionType::stand;
  }

  /**
   * Calculates control based purely on a phase variable
   * phase is a number from 0 to 2*M_PI
   *
   * WARNING !!!!
   * The method robot_->acquire_sensors() has to be called prior to this
   * function call in order to compute the control with up to date sensor data
   */
  void calc_control();

  /**
   * setters for private variables
   */
  void set_phase(double phase) { phase_ = phase; };
  void set_motion_type(MotionType motion_type) { motion_type_ = motion_type; };

 private:
  double phase_;
  MotionType motion_type_;
};