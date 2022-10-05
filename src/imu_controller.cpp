#include "imu_controller.hpp"

using namespace solo;

void IMUController::calc_control() {
  Vector8d desired_positions;

  switch (motion_type_) {
    case MotionType::stand: {
      desired_positions << M_PI / 4, -M_PI / 2, M_PI / 4, -M_PI / 2, -M_PI / 4,
          M_PI / 2, -M_PI / 4, M_PI / 2;
      break;
    }
    case MotionType::tilt_balance: {
      // TODO: rewrite this using external IK implementation
      Eigen::Vector3d imu_attitude = robot_->get_imu_attitude();
      desired_positions << M_PI / 4, -M_PI / 2, M_PI / 4, -M_PI / 2, M_PI / 4,
          -M_PI / 2, M_PI / 4, -M_PI / 2;

      double l_body = 0.39;
      double l_thigh = 0.165;
      double l_calf = 0.165;

      Eigen::Vector2d p_front_des;
      p_front_des << 0.0, -0.25 - l_body / 2.0 * sin(-imu_attitude(1));
      p_front_des << Eigen::Rotation2Dd(-imu_attitude(1)).toRotationMatrix() *
                         p_front_des;
      p_front_des << p_front_des(1), -p_front_des(0);

      double q1 = acos((pow(p_front_des(0), 2) + pow(p_front_des(1), 2) -
                        pow(l_thigh, 2) - pow(l_calf, 2)) /
                       (2 * l_thigh * l_calf));
      double q0 = atan(p_front_des(1) / p_front_des(0)) -
                  atan(l_calf * sin(q1) / (l_thigh + l_calf * cos(q1)));
      q0 *= -1.0;
      q1 *= -1.0;

      Eigen::Vector2d p_rear_des;
      p_rear_des << 0.0, -0.25 + l_body / 2.0 * sin(-imu_attitude(1));
      p_rear_des << Eigen::Rotation2Dd(-imu_attitude(1)).toRotationMatrix() *
                        p_rear_des;
      p_rear_des << p_rear_des(1), -p_rear_des(0);

      double q5 = acos((pow(p_rear_des(0), 2) + pow(p_rear_des(1), 2) -
                        pow(l_thigh, 2) - pow(l_calf, 2)) /
                       (2 * l_thigh * l_calf));
      double q4 = atan(p_rear_des(1) / p_rear_des(0)) -
                  atan(l_calf * sin(q5) / (l_thigh + l_calf * cos(q5)));
      q4 *= -1.0;
      q5 *= -1.0;

      desired_positions(0) = q0;
      desired_positions(1) = q1;
      desired_positions(2) = q0;
      desired_positions(3) = q1;
      desired_positions(4) = q4;
      desired_positions(5) = q5;
      desired_positions(6) = q4;
      desired_positions(7) = q5;
      break;
    }
  }

  desired_positions_ = desired_positions;
}