/**
 * Common header used throughout project
 */

#pragma once

#include <Eigen/Eigen>

namespace solo {
/**
 * @brief Vector2d shortcut for the eigen vector of size 1.
 */
typedef Eigen::Matrix<double, 1, 1> Vector1d;

/**
 * @brief Vector2d shortcut for the eigen vector of size 2.
 */
typedef Eigen::Matrix<double, 2, 1> Vector2d;

/**
 * @brief Vector2d shortcut for the eigen vector of size 6.
 */
typedef Eigen::Matrix<double, 6, 1> Vector6d;

/**
 * @brief Vector8d shortcut for the eigen vector of size 8.
 */
typedef Eigen::Matrix<double, 8, 1> Vector8d;

/**
 * @brief Vector8d shortcut for the eigen vector of size 12.
 */
typedef Eigen::Matrix<double, 12, 1> Vector12d;

}  // namespace solo