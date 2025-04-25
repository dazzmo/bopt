#pragma once

#include <Eigen/Core>

namespace bopt {

using Real = double;
using Index = unsigned long long;

/**
 * @brief Infinity
 *
 */
constexpr double kInf = std::numeric_limits<double>::infinity();

/**
 * @brief Machine precision epsilon
 *
 */
constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

/**
 * @brief Performs a series of checks on a given vector, such as ensuring all
 * values are finite and no NaN are present.
 *
 * @param x
 * @return true Vector is valid
 * @return false Vector is invalid
 */
bool checkVector(const Eigen::Ref<const Eigen::VectorXd> &x);

/**
 * @brief Performs a series of checks on a given matrix, such as ensuring all
 * values are finite and no NaN are present. Also ensures the matrix is not
 * empty.
 *
 * @param x
 * @return true Matrix is valid
 * @return false Matrix is invalid
 */
bool checkMatrix(const Eigen::Ref<const Eigen::MatrixXd> &M);

}  // namespace bopt