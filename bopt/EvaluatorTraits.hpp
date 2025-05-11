#pragma once

#include "bopt/MathTypes.hpp"

namespace bopt {

enum class FunctionType {
    /// @brief Function uses dense matrix and vector types for evaluation
    DENSE,
    /// @brief Function uses sparse matrix and vector types for evaluation
    SPARSE
};

/**
 * @brief Function traits for any function with vector inputs
 *
 * @tparam ScalarType
 */
template <typename ScalarType>
struct EvaluatorTraits {
    using Scalar = ScalarType;

    using DenseVector = typename MathTypes<Scalar>::VectorX;

    using InputVector = DenseVector;
    using InputVectorConstRef = Eigen::Ref<const InputVector>;
};

/**
 * @brief Dense function traits
 *
 * @tparam ScalarType
 */
template <typename ScalarType>
struct DenseEvaluatorTraits : public EvaluatorTraits<ScalarType> {
    using Base = EvaluatorTraits<ScalarType>;

    using Scalar = typename Base::Scalar;
    using InputVector = typename Base::InputVector;
    using InputVectorConstRef = typename Base::InputVectorConstRef;

    using VectorType = typename MathTypes<Scalar>::VectorX;
    using MatrixType = typename MathTypes<Scalar>::MatrixX;

    static constexpr FunctionType type = FunctionType::DENSE;
};

/**
 * @brief Sparse function traits
 *
 * @tparam ScalarType
 */
template <typename ScalarType>
struct SparseEvaluatorTraits : public EvaluatorTraits<ScalarType> {
    using Base = EvaluatorTraits<ScalarType>;

    using Scalar = typename Base::Scalar;
    using InputVector = typename Base::InputVector;
    using InputVectorConstRef = typename Base::InputVectorConstRef;

    using VectorType = typename MathTypes<Scalar>::SparseVector;
    using MatrixType = typename MathTypes<Scalar>::SparseMatrix;

    static constexpr FunctionType type = FunctionType::SPARSE;
};

}  // namespace bopt