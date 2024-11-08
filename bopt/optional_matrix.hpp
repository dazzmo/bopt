#ifndef OPTIONAL_MATRIX_HPP
#define OPTIONAL_MATRIX_HPP

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <optional>

#include "bopt/types.hpp"

namespace bopt {

template <typename Derived, typename MatrixType = Eigen::MatrixBase<Derived>>
class OptionalEigenMatrix {
   public:
    OptionalEigenMatrix()
        : map_(nullptr, MatrixType::RowsAtCompileTime,
               MatrixType::ColsAtCompileTime) {}

    OptionalEigenMatrix(std::nullptr_t)
        : map_(nullptr, MatrixType::RowsAtCompileTime,
               MatrixType::ColsAtCompileTime) {}
    OptionalEigenMatrix(std::nullopt_t)
        : map_(nullptr, MatrixType::RowsAtCompileTime,
               MatrixType::ColsAtCompileTime) {}

    // Create from structured matrix
    template <int _Rows, int _Cols>
    OptionalEigenMatrix(Eigen::Matrix<Scalar, _Rows, _Cols> &M)
        : map_(nullptr, M.rows(), M.cols()) {
        new (&map_) Eigen::Map<Derived>(M.data(), M.rows(), M.cols());
    }

    OptionalEigenMatrix(Scalar &val)
        : map_(nullptr, MatrixType::RowsAtCompileTime,
               MatrixType::ColsAtCompileTime) {
        new (&map_) Eigen::Map<Derived>(&val, 1, 1);
    }

    template <typename Other>
    OptionalEigenMatrix(OptionalEigenMatrix<Other> &M)
        : map_(nullptr, Other::RowsAtCompileTime, Other::ColsAtCompileTime) {
        new (&map_) Eigen::Map<Derived>(M->data(), M->rows(), M->cols());
    }

    operator bool() const { return map_.data() != nullptr; }

    Eigen::Map<Derived> *operator->() { return &map_; }
    Eigen::Map<Derived> &operator*() { return map_; }

    const Eigen::Map<Derived> *operator->() const { return &map_; }
    const Eigen::Map<Derived> &operator*() const { return map_; }

   private:
    Eigen::Map<Derived> map_;
};

// /**
//  * @brief A class representing an optional matrix argument for a function,
//  which can be included optionally or ignored by using setting the default
//  value to its null constructor.
//  *
//  * @tparam Rows
//  * @tparam Cols
//  *
//  */
// template <typename _Scalar, int _Rows, int MatrixType::ColsAtCompileTime>
// class OptionalMatrixData {
//    public:
//     typedef Eigen::Matrix<double, _Rows, _Cols> MatrixType;

//     OptionalMatrixData() : map_(nullptr, _Rows, _Cols) {}

//     OptionalMatrixData(std::nullptr_t) : map_(nullptr, _Rows, _Cols) {}
//     OptionalMatrixData(std::nullopt_t) : map_(nullptr, _Rows, _Cols) {}

//     // Create from structured matrix
//     template <int U, int V>
//     OptionalMatrixData(Eigen::Matrix<double, U, V>& M)
//         : map_(nullptr, _Rows, _Cols) {
//         new (&map_) Eigen::Map<MatrixType>(M.data(), M.rows(), M.cols());
//     }

//     // Create from structured matrix
//     template <int U, int V>
//     OptionalMatrixData(OptionalMatrixData<U, V>& M)
//         : map_(nullptr, _Rows, _Cols) {
//         new (&map_) Eigen::Map<MatrixType>(M->data(), M->rows(), M->cols());
//     }

//     OptionalMatrixData(double& val) : map_(nullptr, _Rows, _Cols) {
//         new (&map_) Eigen::Map<MatrixType>(&val, 1, 1);
//     }

//     operator bool() const { return map_.data() != nullptr; }

//     Eigen::Map<MatrixType>* operator->() { return &map_; }
//     Eigen::Map<MatrixType>& operator*() { return map_; }

//     const Eigen::Map<MatrixType>* operator->() const { return &map_; }
//     const Eigen::Map<MatrixType>& operator*() const { return map_; }

//    private:
//     Eigen::Map<MatrixType> map_;
// };

// using OptionalVector = OptionalMatrixData<Scalar, -1, 1>;
// using OptionalRowVector = OptionalMatrixData<Scalar, 1, -1>;
// using OptionalMatrix = OptionalMatrixData<Scalar, -1, -1>;

}  // namespace bopt
#endif /* OPTIONAL_MATRIX_HPP */
