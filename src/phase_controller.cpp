#include "phase_controller.hpp"

using namespace solo;

void PhaseController::calc_control() {
  Vector8d desired_positions;

  switch (motion_type_) {
    case MotionType::stand: {
      desired_positions << M_PI / 4, -M_PI / 2, M_PI / 4, -M_PI / 2, -M_PI / 4,
          M_PI / 2, -M_PI / 4, M_PI / 2;
      break;
    }
    case MotionType::squat: {
      desired_positions << M_PI / 4, -M_PI / 2, M_PI / 4, -M_PI / 2, -M_PI / 4,
          M_PI / 2, -M_PI / 4, M_PI / 2;
      double amp = M_PI / 8.0;
      desired_positions(0) += amp * sin(phase_);
      desired_positions(1) += -2.0 * amp * sin(phase_);
      desired_positions(2) += amp * sin(phase_);
      desired_positions(3) += -2.0 * amp * sin(phase_);
      desired_positions(4) -= amp * sin(phase_);
      desired_positions(5) -= -2.0 * amp * sin(phase_);
      desired_positions(6) -= amp * sin(phase_);
      desired_positions(7) -= -2.0 * amp * sin(phase_);
      break;
    }
    case MotionType::tilt_body: {
      desired_positions << M_PI / 4, -M_PI / 2, M_PI / 4, -M_PI / 2, -M_PI / 4,
          M_PI / 2, -M_PI / 4, M_PI / 2;
      double amp = M_PI / 20.0;
      desired_positions(0) += amp * sin(phase_);
      desired_positions(1) += -2.0 * amp * sin(phase_);
      desired_positions(2) += amp * sin(phase_);
      desired_positions(3) += -2.0 * amp * sin(phase_);
      desired_positions(4) += amp * sin(phase_);
      desired_positions(5) += -2.0 * amp * sin(phase_);
      desired_positions(6) += amp * sin(phase_);
      desired_positions(7) += -2.0 * amp * sin(phase_);
      break;
    }
    case MotionType::step_in_place: {
      desired_positions << M_PI / 4, -M_PI / 2, M_PI / 4, -M_PI / 2, -M_PI / 4,
          M_PI / 2, -M_PI / 4, M_PI / 2;
      double amp = M_PI / 8.0;

      desired_positions(0) += std::max(amp * sin(phase_), 0.0);
      desired_positions(1) += std::min(-2.0 * amp * sin(phase_), 0.0);
      desired_positions(2) += std::max(amp * sin(phase_ + M_PI), 0.0);
      desired_positions(3) += std::min(-2.0 * amp * sin(phase_ + M_PI), 0.0);
      desired_positions(4) += std::min(amp * sin(phase_), 0.0);
      desired_positions(5) += std::max(-2.0 * amp * sin(phase_), 0.0);
      desired_positions(6) += std::min(amp * sin(phase_ + M_PI), 0.0);
      desired_positions(7) += std::max(-2.0 * amp * sin(phase_ + M_PI), 0.0);
      break;
    }
    case MotionType::walk: {
      Vector8d p;

      double offset = -0.25;
      double amp = 0.07;
      double amp_x = 0.07 * std::sin(0.04 * phase_);

      double phase_mod1 = std::fmod(phase_, 2.0 * M_PI);
      double phase_mod2 = std::fmod(phase_ + M_PI, 2.0 * M_PI);

      p(1) = offset + std::max(amp * sin(phase_), 0.0);
      p(3) = offset + std::max(amp * sin(phase_ + M_PI), 0.0);
      p(5) = offset + std::max(amp * sin(phase_ + M_PI), 0.0);
      p(7) = offset + std::max(amp * sin(phase_), 0.0);

      p(0) = phase_mod2 >= M_PI
                 ? -amp_x * (M_PI - phase_mod1) / (M_PI - 0.0) +
                       amp_x * (phase_mod1 - 0.0) / (M_PI - 0.0)
                 : amp_x * (2.0 * M_PI - phase_mod1) / (2.0 * M_PI - M_PI) +
                       -amp_x * (phase_mod1 - M_PI) / (2.0 * M_PI - M_PI);
      p(2) = phase_mod1 >= M_PI
                 ? -amp_x * (M_PI - phase_mod2) / (M_PI - 0.0) +
                       amp_x * (phase_mod2 - 0.0) / (M_PI - 0.0)
                 : amp_x * (2.0 * M_PI - phase_mod2) / (2.0 * M_PI - M_PI) +
                       -amp_x * (phase_mod2 - M_PI) / (2.0 * M_PI - M_PI);
      p(4) = phase_mod1 >= M_PI
                 ? -amp_x * (M_PI - phase_mod2) / (M_PI - 0.0) +
                       amp_x * (phase_mod2 - 0.0) / (M_PI - 0.0)
                 : amp_x * (2.0 * M_PI - phase_mod2) / (2.0 * M_PI - M_PI) +
                       -amp_x * (phase_mod2 - M_PI) / (2.0 * M_PI - M_PI);
      p(6) = phase_mod2 >= M_PI
                 ? -amp_x * (M_PI - phase_mod1) / (M_PI - 0.0) +
                       amp_x * (phase_mod1 - 0.0) / (M_PI - 0.0)
                 : amp_x * (2.0 * M_PI - phase_mod1) / (2.0 * M_PI - M_PI) +
                       -amp_x * (phase_mod1 - M_PI) / (2.0 * M_PI - M_PI);

      bool elbow_up[4] = {true, true, false, false};

      desired_positions << solo_IK(p, elbow_up);
      break;
    }
  }

  desired_positions_ = desired_positions;
}