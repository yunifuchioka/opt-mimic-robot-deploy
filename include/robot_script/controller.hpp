/**
 * Abstract class for a controller
 * Inputs: robot sensor data, other miscellaneous inputs (eg. time)
 * Outputs: PD+torque targets for the robot
 * All IO is done through private variables and getter/setter functions
 */

#pragma once

#include "solo8.hpp"

using namespace solo;

class Controller {
 public:
  /**
   * constructor to initialize internal variables
   */
  Controller(const std::shared_ptr<Solo8>& robot) {
    robot_ = robot;
    desired_positions_.setZero();
    desired_velocities_.setZero();
    desired_torques_.setZero();
  }

  /**
   * reads sensor data and updates internal PD+torque target variables
   * Should be implemented for each child class
   *
   * WARNING !!!!
   * The method robot_->acquire_sensors() has to be called prior to this
   * function call in order to compute the control with up to date sensor data
   */
  virtual void calc_control() = 0;

  /**
   * Getters for PD+torque targets computed by calc_control()
   */
  const Eigen::Ref<Vector8d> get_desired_positions() {
    return desired_positions_;
  }
  const Eigen::Ref<Vector8d> get_desired_velocities() {
    return desired_velocities_;
  }
  const Eigen::Ref<Vector8d> get_desired_torques() { return desired_torques_; }

 protected:
  /**
   * keep a pointer to the robot in order to read off sensor data
   * the intention is not for any commands to be sent to the robot from this
   * class--that should happen in the consumer code (eg main control loop)
   */
  std::shared_ptr<Solo8> robot_;

  /**
   * stores PD+Torque targets
   */
  Vector8d desired_positions_;
  Vector8d desired_velocities_;
  Vector8d desired_torques_;

  /**
   * HELPER FUNCTIONS
   */

  /**
   * generic planar 2 link inverse kinematics implementation
   * returns the closest point within the workspace if the requested point is
   * outside of it
   */
  const Vector2d planar_IK(double l1, double l2, double x, double y,
                           bool elbow_up) {
    double l;
    double alpha;
    double cos_beta;
    double beta;
    double cos_th2_abs;
    double th2_abs;
    Vector2d th;

    l = std::sqrt(x * x + y * y);
    l = std::max(std::abs(l1 - l2), std::min(l, l1 + l2));

    alpha = std::atan2(y, x);

    cos_beta = (l * l + l1 * l1 - l2 * l2) / (2.0 * l * l1);
    cos_beta = std::max(-1.0, std::min(cos_beta, 1.0));
    beta = std::acos(cos_beta);

    cos_th2_abs = (l * l - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
    cos_th2_abs = std::max(-1.0, std::min(cos_th2_abs, 1.0));
    th2_abs = std::acos(cos_th2_abs);

    if (elbow_up) {
      th << alpha - beta, th2_abs;
    } else {
      th << alpha + beta, -th2_abs;
    }

    return th;
  }

  /**
   * inverse kinematics for the solo 8 robot
   */
  Vector8d solo_IK(const Vector8d p, bool* elbow_up) {
    Eigen::Matrix2d rotate_90;
    rotate_90 << 0.0, -1.0, 1.0, 0.0;

    Vector2d p_curr;
    Vector2d x_y_curr;
    Vector2d th_curr;
    Vector8d q;

    for (unsigned int leg_idx = 0; leg_idx < 4; leg_idx++) {
      p_curr = p.segment(leg_idx * 2, 2);
      x_y_curr = rotate_90 * p_curr;
      th_curr =
          planar_IK(0.165, 0.160, x_y_curr(0), x_y_curr(1), elbow_up[leg_idx]);
      th_curr *= -1.0;
      q.segment(leg_idx * 2, 2) = th_curr;
    }

    return q;
  }
};