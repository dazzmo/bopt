#pragma once

#include "bopt/MathTypes.hpp"

namespace bopt {

enum class SparsityType {
    /// @brief Function uses dense matrix and vector types for evaluation
    DENSE,
    /// @brief Function uses sparse matrix and vector types for evaluation
    SPARSE
};

template <typename ScalarType, int OutputSizeAtCompileTime = Eigen::Dynamic,
          SparsityType Sparsity = SparsityType::DENSE>
struct EvaluatorTraits {};

/**
 * @brief Evaluator traits for functions that output dense Jacobians and
 * Hessians
 *
 * @tparam ScalarType
 * @tparam OutputSizeAtCompileTime
 */
template <typename ScalarType, int OutputSizeAtCompileTime>
struct EvaluatorTraits<ScalarType, OutputSizeAtCompileTime,
                       SparsityType::DENSE> {
    using Scalar = ScalarType;

    using DenseVector = typename MathTypes<Scalar>::VectorX;

    using InputVectorType = DenseVector;

    using OutputType = DenseVector;
    using OutputVectorType = typename MathTypes<Scalar>::VectorX;
    using OutputMatrixType = typename MathTypes<Scalar>::MatrixX;

    using OutputJacobianType = OutputMatrixType;
    using OutputHessianType = OutputMatrixType;
};

/**
 * @brief
 *
 * @tparam ScalarType
 */
template <typename ScalarType>
struct EvaluatorTraits<ScalarType, 1, SparsityType::DENSE> {
    using Scalar = ScalarType;

    using DenseVector = typename MathTypes<Scalar>::VectorX;

    using InputVectorType = DenseVector;

    using OutputType = Scalar;
    using OutputVectorType = typename MathTypes<Scalar>::VectorX;
    using OutputMatrixType = typename MathTypes<Scalar>::MatrixX;

    using OutputGradientType = DenseVector;
    using OutputHessianType = OutputMatrixType;
};

/**
 * @brief Sparse function traits
 *
 * @tparam ScalarType
 */
template <typename ScalarType, int OutputSizeAtCompileTime>
struct EvaluatorTraits<ScalarType, OutputSizeAtCompileTime,
                       SparsityType::SPARSE> {
    using Scalar = ScalarType;

    using DenseVector = typename MathTypes<Scalar>::VectorX;

    using InputVectorType = DenseVector;
    
    using OutputType = DenseVector;
    using OutputVectorType = typename MathTypes<Scalar>::SparseVector;
    using OutputMatrixType = typename MathTypes<Scalar>::SparseMatrix;

    using OutputJacobianType = OutputMatrixType;
    using OutputHessianType = OutputMatrixType;
};

template <typename ScalarType>
struct EvaluatorTraits<ScalarType, 1, SparsityType::SPARSE> {
    using Scalar = ScalarType;

    using DenseVector = typename MathTypes<Scalar>::VectorX;

    using InputVectorType = DenseVector;

    using OutputType = Scalar;
    using OutputVectorType = typename MathTypes<Scalar>::SparseVector;
    using OutputMatrixType = typename MathTypes<Scalar>::SparseMatrix;

    using OutputGradientType = OutputVectorType;
    using OutputHessianType = OutputMatrixType;
};

}  // namespace bopt