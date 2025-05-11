#pragma once
#include <Eigen/Core>
#include <Eigen/Sparse>

#include "bopt/Types.hpp"

namespace bopt {

/**
 * @brief Math typings within the Eigen library.
 *
 * @tparam T
 */
template <typename T>
struct MathTypes {
    template <int Size>
    using Vector = Eigen::Vector<T, Size>;

    template <int Rows, int Cols>
    using Matrix = Eigen::Matrix<T, Rows, Cols>;

    using VectorX = Eigen::VectorX<T>;
    using MatrixX = Eigen::MatrixX<T>;

    using SparseMatrix = Eigen::SparseMatrix<T>;
    using SparseVector = Eigen::SparseVector<T>;
};

}  // namespace bopt
