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
    using VectorX = Eigen::VectorX<T>;
    using MatrixX = Eigen::MatrixX<T>;

    using SparseMatrix = Eigen::SparseMatrix<T>;
    using SparseVector = Eigen::SparseVector<T>;
};

}  // namespace bopt
