/**
 * controller that uses a neural network trained by RL
 */

#pragma once

#include "controller.hpp"
#include "torch/script.h"

#define NETWORK_INPUT_DIM 22
#define HORIZON 3
#define SENSOR_DIM 20
#define ACTION_DIM 8
// #define NETWORK_INPUT_DIM (HORIZON + 1) * SENSOR_DIM + HORIZON* ACTION_DIM + 2
#define NETWORK_OUTPUT_DIM 8

class NetworkController : public Controller {
 public:
  enum ControllerState { homing, stand, fold, motion };
  typedef Eigen::Matrix<double, NETWORK_INPUT_DIM, 1> VectorObservation;
  typedef Eigen::Matrix<double, NETWORK_OUTPUT_DIM, 1> VectorAction;

  /**
   * constructor calls the parent controller constructor, then initializes its
   * own internal variables
   */
  NetworkController(const std::shared_ptr<Solo8>& robot) : Controller{robot} {
    controllerState_ = ControllerState::homing;
    time_ = 0.0;
    time_stamp_state_change_ = 0.0;
    ref_traj_.setZero();
    ref_traj_max_idx_ = 0;
    ref_traj_max_time_ = 0.0;
    desired_positions_reference_.setZero();
    desired_velocities_reference_.setZero();
    desired_torques_reference_.setZero();
    filtered_velocity_.setZero();
    sensor_history_.setZero(HORIZON * SENSOR_DIM);
    action_history_.setZero(HORIZON * ACTION_DIM);
    // TODO: initialize network to something safe, prior to initialize_network
    // call
  }

  /**
   * Loads neural network model given by the specified filename
   */
  void initialize_network(const std::string filename);

  /**
   * Calculates control based phase variable and sensor data from the robot
   *
   * WARNING !!!!
   * The method robot_->acquire_sensors() has to be called prior to this
   * function call in order to compute the control with up to date sensor data
   */
  void calc_control();

  /**
   * setters for private variables
   */
  void set_time(double time) { time_ = time; };
  void set_traj(const Eigen::MatrixXd ref_traj) {
    ref_traj_ = ref_traj;
    ref_traj_max_idx_ = ref_traj_.rows() - 1;
    ref_traj_max_time_ = ref_traj(ref_traj_max_idx_, 0);
  };
  void set_filtered_velocity(const Vector8d filtered_velocity) {
    filtered_velocity_ = filtered_velocity;
  };

 private:
  ControllerState controllerState_;
  double time_;
  double time_stamp_state_change_;
  torch::jit::script::Module network_;
  Vector8d desired_positions_reference_;
  Vector8d desired_velocities_reference_;
  Vector8d desired_torques_reference_;
  Eigen::MatrixXd ref_traj_;
  int ref_traj_max_idx_;
  double ref_traj_max_time_;
  Vector8d filtered_velocity_;
  Eigen::VectorXd sensor_history_;
  Eigen::VectorXd action_history_;

  void setReferenceMotionTraj(const int traj_idx);
};