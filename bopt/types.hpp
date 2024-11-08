
#include <Eigen/Core>
#include <iostream>

namespace bopt {

using Index = std::size_t;

using Scalar = double;

template <int _Rows>
using Vector = Eigen::Vector<Scalar, _Rows>;

typedef Vector<2> Vector2;
typedef Vector<3> Vector3;
typedef Vector<Eigen::Dynamic> VectorX;

template <int _Rows>
using RowVector = Eigen::RowVector<Scalar, _Rows>;

typedef RowVector<Eigen::Dynamic> RowVectorX;

template <int _Rows, int _Cols>
using Matrix = Eigen::Matrix<Scalar, _Rows, _Cols>;

typedef Matrix<2, 2> Matrix2;
typedef Matrix<3, 3> Matrix3;
typedef Matrix<Eigen::Dynamic, Eigen::Dynamic> MatrixX;

}  // namespace bopt
