
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <iostream>
#include <unsupported/Eigen/AutoDiff>
// Include later?
#include <cppad/example/cppad_eigen.hpp>

namespace bopt {

// New
using Real = double;

using Index = unsigned long long;

using bopt_int = long long;
using bopt_double = double;
using bopt_index = std::size_t;

template <typename T>
using VectorX = Eigen::VectorX<T>;

template <typename T>
using MatrixX = Eigen::MatrixX<T>;

using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using VectorXd = Eigen::VectorXd;

using Matrix3d = Eigen::Matrix3d;
using MatrixXd = Eigen::MatrixXd;

template <typename Scalar>
using SparseMatrix = Eigen::SparseMatrix<Scalar>;

template <typename Scalar>
using SparseVector = Eigen::SparseVector<Scalar>;

using AD = Eigen::AutoDiffScalar<VectorXd>;
using VectorXAD = Eigen::VectorX<AD>;
using ADD = Eigen::AutoDiffScalar<VectorXAD>;
using VectorXADD = VectorX<ADD>;

using VectorXCPPAD = Eigen::VectorX<CppAD::AD<double>>;

}  // namespace bopt
