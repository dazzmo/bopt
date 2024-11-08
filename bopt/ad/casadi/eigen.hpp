#ifndef UTILS_EIGEN_WRAPPER_H
#define UTILS_EIGEN_WRAPPER_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <casadi/casadi.hpp>
#include <string>
#include <vector>

// namespace boost {
// namespace math {
// namespace constants {
// namespace detail {
// template <>
// struct constant_pi<::casadi::SX> : constant_pi<double> {};

// template <typename Scalar>
// struct constant_pi<::casadi::Matrix<Scalar>> : constant_pi<Scalar> {};
// }  // namespace detail
// }  // namespace constants
// }  // namespace math
// }  // namespace boost

// todo - Taken from Pinocchio, acknowledge

// This is a workaround to make the code compiling with Eigen.
namespace casadi {
inline bool operator||(const bool x, const casadi::Matrix<SXElem> & /*y*/) {
  return x;
}
}  // namespace casadi

namespace Eigen {
namespace internal {
// Specialization of Eigen::internal::cast_impl for Casadi input types
template <typename Scalar>
struct cast_impl<casadi::SX, Scalar> {
#if EIGEN_VERSION_AT_LEAST(3, 2, 90)
  EIGEN_DEVICE_FUNC
#endif
  static inline Scalar run(const casadi::SX &x) {
    return static_cast<Scalar>(x);
  }
};

#if EIGEN_VERSION_AT_LEAST(3, 2, 90) && !EIGEN_VERSION_AT_LEAST(3, 2, 93)
template <typename Scalar, bool IsInteger>
struct significant_decimals_default_impl<::casadi::Matrix<Scalar>, IsInteger> {
  static inline int run() { return std::numeric_limits<Scalar>::digits10; }
};
#endif
}  // namespace internal
}  // namespace Eigen

namespace Eigen {
/// @brief Eigen::NumTraits<> specialization for casadi::SX
///
template <typename Scalar>
struct NumTraits<casadi::Matrix<Scalar>> {
  using Real = casadi::Matrix<Scalar>;
  using NonInteger = casadi::Matrix<Scalar>;
  using Literal = casadi::Matrix<Scalar>;
  using Nested = casadi::Matrix<Scalar>;

  enum {
    // does not support complex Base types
    IsComplex = 0,
    // does not support integer Base types
    IsInteger = 0,
    // only support signed Base types
    IsSigned = 1,
    // must initialize an AD<Base> object
    RequireInitialization = 1,
    // computational cost of the corresponding operations
    ReadCost = 1,
    AddCost = 2,
    MulCost = 2
  };

  static casadi::Matrix<Scalar> epsilon() {
    return casadi::Matrix<Scalar>(std::numeric_limits<double>::epsilon());
  }

  static casadi::Matrix<Scalar> dummy_precision() {
    return casadi::Matrix<Scalar>(NumTraits<double>::dummy_precision());
  }

  static casadi::Matrix<Scalar> highest() {
    return casadi::Matrix<Scalar>(std::numeric_limits<double>::max());
  }

  static casadi::Matrix<Scalar> lowest() {
    return casadi::Matrix<Scalar>(std::numeric_limits<double>::min());
  }

  static int digits10() { return std::numeric_limits<double>::digits10; }
};
}  // namespace Eigen

namespace bopt {
namespace casadi {

/**
 * @brief Convert a casadi::Matrix<T> matric to an Eigen::Matrix<T>
 *
 * @tparam T
 * @tparam rows
 * @tparam cols
 * @param C
 * @param E
 */
template <typename T, int rows, int cols>
void toEigen(const ::casadi::Matrix<T> &C, Eigen::Matrix<T, rows, cols> &E) {
  E.setZero(C.rows(), C.columns());
  for (casadi_int i = 0; i < C.rows(); ++i) {
    for (casadi_int j = 0; j < C.columns(); ++j) {
      E(i, j) = T(C(i, j));
    }
  }
}

/**
 * @brief Convert an Eigen::Matrix<T> object to a casadi::Matrix<T> object
 *
 * @tparam T
 * @tparam rows
 * @tparam cols
 * @param E
 * @param C
 */
template <typename T, int rows, int cols>
void toCasadi(const Eigen::Matrix<T, rows, cols> &E, ::casadi::Matrix<T> &C) {
  C.resize(E.rows(), E.cols());
  for (int i = 0; i < E.rows(); ++i) {
    for (int j = 0; j < E.cols(); ++j) {
      // Only fill in non-zero entries
      if (!::casadi::is_zero(E(i, j))) {
        C(i, j) = E(i, j);
      }
    }
  }
}

/**
 * @brief Convert a casadi::Matrix<T> object to an
 * Eigen::Matrix<casadi::Matrix<T>> object (e.g. convert a casadi::SX to an
 * Eigen::Matrix<casadi::SX>)
 *
 * @tparam T
 * @tparam rows
 * @tparam cols
 * @param C
 * @param E
 */
template <typename T, int rows, int cols>
void toEigen(const ::casadi::Matrix<T> &C,
             Eigen::Matrix<::casadi::Matrix<T>, rows, cols> &E) {
  E.setZero(C.rows(), C.columns());
  for (casadi_int i = 0; i < C.rows(); ++i) {
    for (casadi_int j = 0; j < C.columns(); ++j) {
      E(i, j) = ::casadi::Matrix<T>(C(i, j));
    }
  }
}

template <typename T, int rows, int cols>
void toCasadi(const Eigen::Matrix<::casadi::Matrix<T>, rows, cols> &E,
              ::casadi::Matrix<T> &C) {
  C.resize(E.rows(), E.cols());
  for (int i = 0; i < E.rows(); ++i) {
    for (int j = 0; j < E.cols(); ++j) {
      // Only fill in non-zero entries
      if (!::casadi::is_zero(E(i, j)->at(0))) {
        C(i, j) = E(i, j)->at(0);
      }
    }
  }
}

/**
 * @brief Create a casadi::Function object using Eigen vectors containing
 * casadi::SX elements.
 *
 * @param name
 * @param in
 * @param out
 * @return ::casadi::Function
 */
::casadi::Function toCasadiFunction(
    const std::string &name,
    const std::vector<Eigen::VectorX<::casadi::SX>> &in,
    const std::vector<Eigen::VectorX<::casadi::SX>> &out);

}  // namespace casadi
}  // namespace bopt

#endif /* UTILS_EIGEN_WRAPPER_H */
