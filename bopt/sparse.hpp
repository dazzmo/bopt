#include <vector>

#include "bopt/evaluator.hpp"
#include "bopt/logging.hpp"

namespace bopt {

struct BinaryOpSet {
    template <typename T, typename U>
    auto operator()(T, U value) const {
        return value;  // Set the value
    }
};

struct BinaryOpAddTo {
    template <typename T, typename U>
    auto operator()(T existing, U value) const {
        return existing + value;
    }
};

template <class ValueType, class BinaryOp>
void insert_block(Eigen::MatrixX<ValueType> &matrix,
                  const Eigen::MatrixX<ValueType> &block,
                  const std::vector<Eigen::Index> &row_indices,
                  const std::vector<Eigen::Index> &col_indices, BinaryOp op) {
    assert(row_indices.size() == block.rows() &&
           col_indices.size() == block.cols() &&
           "Indices provided are not same size as provided block");

    // Populate matrix in a dense manner
    for (Eigen::Index col = 0; col < block.cols(); ++col) {
        for (Eigen::Index row = 0; row < block.rows(); ++row) {
            VLOG(10) << "Accessing (" << row_indices[row] << ", "
                     << col_indices[col] << ") to insert " << block(row, col);
            matrix(row, col) = op(matrix(row, col), block(row, col));
        }
    }
}

template <class ValueType, class BinaryOp>
void insert_block(Eigen::SparseMatrix<ValueType> &matrix,
                  const Eigen::MatrixX<ValueType> &block,
                  const std::vector<Eigen::Index> &row_indices,
                  const std::vector<Eigen::Index> &col_indices, BinaryOp op) {
    assert(row_indices.size() == block.rows() &&
           col_indices.size() == block.cols() &&
           "Indices provided are not same size as provided block");

    // Populate matrix in a dense manner
    for (Eigen::Index col = 0; col < block.cols(); ++col) {
        for (Eigen::Index row = 0; row < block.rows(); ++row) {
            VLOG(10) << "Accessing (" << row_indices[row] << ", "
                     << col_indices[col] << ") to insert "
                     << block_data.values[col * block.rows() + row];
            matrix.coeffRef(row_indices[row], col_indices[col]) =
                op(matrix.coeffRef(row_indices[row], col_indices[col]),
                   block(row, col));
        }
    }
}

template <class ValueType, class BinaryOp>
void insert_block(Eigen::MatrixX<ValueType> &matrix,
                  const Eigen::SparseMatrix<ValueType> &block,
                  const std::vector<Eigen::Index> &row_indices,
                  const std::vector<Eigen::Index> &col_indices, BinaryOp op) {
    assert(row_indices.size() == block.rows() &&
           col_indices.size() == block.cols() &&
           "Indices provided are not same size as provided block");

    // Iterate over the sparse block and insert the correct entries
    for (int k = 0; k < block.outerSize(); ++k) {
        for (Eigen::SparseMatrix<ValueType>::InnerIterator it(block, k); it;
             ++it) {
            matrix(row_indices[it.row()], col_indices[it.col()]) =
                op(matrix(row_indices[it.row()], col_indices[it.col()]),
                   it.value());
        }
    }
}

template <class ValueType, class BinaryOp>
void insert_block(Eigen::SparseMatrix<ValueType> &matrix,
                  const Eigen::SparseMatrix<ValueType> &block,
                  const std::vector<Eigen::Index> &row_indices,
                  const std::vector<Eigen::Index> &col_indices, BinaryOp op) {
    assert(row_indices.size() == block.rows() &&
           col_indices.size() == block.cols() &&
           "Indices provided are not same size as provided block");

    // Iterate over the sparse block and insert the correct entries
    for (int k = 0; k < block.outerSize(); ++k) {
        for (Eigen::SparseMatrix<ValueType>::InnerIterator it(block, k); it;
             ++it) {
            matrix.coeffRef(row_indices[it.row()], col_indices[it.col()]) = op(
                matrix.coeffRef(row_indices[it.row()], col_indices[it.col()]),
                it.value());
        }
    }
}

}  // namespace bopt